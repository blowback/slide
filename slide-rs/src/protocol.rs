use anyhow::{bail, Result};
use std::thread;
use std::time::{Duration, Instant};

// Protocol constants
pub const SOF: u8 = 0x01;
pub const CTRL_ACK: u8 = 0x06;
pub const CTRL_NAK: u8 = 0x15;
pub const CTRL_RDY: u8 = 0x11;
pub const CTRL_FIN: u8 = 0x04;
pub const CTRL_CAN: u8 = 0x18;
/// v0.3 §2: "a command frame follows" / capability probe.
pub const CTRL_ENQ: u8 = 0x05;

pub const WIN_SIZE: usize = 4;
pub const FRAME_SIZE: usize = 1024;

/// v0.2.1 §1: Z80 emits this 7-byte signature on entering SLIDE mode,
/// before any RDY/control/payload. PCs may use it as a liveness signal;
/// they MUST tolerate it appearing on the wire as non-SOF / non-control bytes.
#[allow(dead_code)]
pub const WAKEUP_SIG: [u8; 7] = [0x1B, 0x5E, b'S', b'L', b'I', b'D', b'E'];

/// v0.2.1 §2: echo CTRL_CAN back to the peer within ~500 ms, then drain
/// the wire so the next session starts clean.
pub fn echo_can_and_drain(port: &mut dyn serialport::SerialPort) -> Result<()> {
    port.write_all(&[CTRL_CAN])?;
    port.flush()?;
    thread::sleep(Duration::from_millis(50));
    port.clear(serialport::ClearBuffer::Input)?;
    Ok(())
}

/// CRC-16-CCITT (polynomial 0x1021, init 0xFFFF).
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
            crc &= 0xFFFF;
        }
    }
    crc
}

/// Build a complete wire frame: [SOF][SEQ][LEN_H][LEN_L][PAYLOAD][CRC_H][CRC_L]
pub fn build_frame(seq: u8, payload: &[u8]) -> Vec<u8> {
    let length = payload.len();
    let mut crc_data = vec![seq, (length >> 8) as u8, (length & 0xFF) as u8];
    crc_data.extend_from_slice(payload);
    let crc = crc16_ccitt(&crc_data);

    let mut frame = vec![SOF, seq, (length >> 8) as u8, (length & 0xFF) as u8];
    frame.extend_from_slice(payload);
    frame.push((crc >> 8) as u8);
    frame.push((crc & 0xFF) as u8);
    frame
}

/// Build header frame: null-terminated filename + 4-byte LE size, seq=0.
pub fn build_header_frame(filename: &str, filesize: u32) -> Vec<u8> {
    let name = std::path::Path::new(filename)
        .file_name()
        .unwrap_or_default()
        .to_string_lossy()
        .to_uppercase();
    let mut payload = name.into_bytes();
    payload.push(0); // null terminator
    payload.extend_from_slice(&filesize.to_le_bytes());
    build_frame(0, &payload)
}

/// Control byte response from the remote side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Control {
    Ack(u8),
    Nak(u8),
    Rdy,
    Fin,
    Can,
}

/// Read exactly one byte with a timeout, returning None on timeout.
pub(crate) fn read_byte_timeout(port: &mut dyn serialport::SerialPort, timeout: Duration) -> Result<Option<u8>> {
    port.set_timeout(timeout)?;
    let mut buf = [0u8; 1];
    match port.read(&mut buf) {
        Ok(1) => Ok(Some(buf[0])),
        Ok(_) => Ok(None),
        Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
        Err(e) => Err(e.into()),
    }
}

/// Wait for a control byte (ACK/NAK/RDY/CAN/FIN) from the remote side.
///
/// v0.2.1 §2: on CTRL_CAN this auto-echoes CAN back and drains the wire
/// before returning Control::Can — caller can simply treat it as a
/// confirmed cancel and abort to idle.
pub fn recv_control(port: &mut dyn serialport::SerialPort, timeout: Duration) -> Result<Control> {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            bail!("Timeout waiting for control byte");
        }
        let Some(b) = read_byte_timeout(port, remaining.min(Duration::from_secs(2)))? else {
            if Instant::now() >= deadline {
                bail!("Timeout waiting for control byte");
            }
            continue;
        };
        match b {
            CTRL_ACK | CTRL_NAK => {
                let remaining = deadline.saturating_duration_since(Instant::now());
                let Some(seq) = read_byte_timeout(port, remaining.min(Duration::from_secs(2)))? else {
                    bail!("Timeout waiting for sequence byte");
                };
                return Ok(if b == CTRL_ACK { Control::Ack(seq) } else { Control::Nak(seq) });
            }
            CTRL_RDY => return Ok(Control::Rdy),
            CTRL_CAN => {
                echo_can_and_drain(port)?;
                return Ok(Control::Can);
            }
            CTRL_FIN => return Ok(Control::Fin),
            _ => continue, // ignore spurious bytes (incl. wakeup signature)
        }
    }
}

/// Received frame data.
pub struct Frame {
    pub seq: u8,
    pub payload: Vec<u8>,
}

/// Result of recv_frame: a data frame, FIN, or peer-initiated cancel
/// (CAN already echoed and wire drained per v0.2.1 §2).
pub enum FrameResult {
    Data(Frame),
    Fin,
    Cancel,
}

/// Receive a SLIDE frame from the serial port.
pub fn recv_frame(port: &mut dyn serialport::SerialPort, timeout: Duration) -> Result<FrameResult> {
    let deadline = Instant::now() + timeout;

    // Wait for SOF, FIN, or CAN. Other bytes (e.g. wakeup signature) are skipped.
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            bail!("Timeout waiting for SOF");
        }
        let Some(b) = read_byte_timeout(port, remaining.min(Duration::from_secs(2)))? else {
            continue;
        };
        if b == SOF {
            break;
        }
        if b == CTRL_FIN {
            return Ok(FrameResult::Fin);
        }
        if b == CTRL_CAN {
            echo_can_and_drain(port)?;
            return Ok(FrameResult::Cancel);
        }
    }

    recv_frame_after_sof(port, deadline)
}

/// Receive frame body after SOF has been consumed.
pub fn recv_frame_after_sof(port: &mut dyn serialport::SerialPort, deadline: Instant) -> Result<FrameResult> {
    let read_byte = |port: &mut dyn serialport::SerialPort| -> Result<u8> {
        let remaining = deadline.saturating_duration_since(Instant::now());
        read_byte_timeout(port, remaining.min(Duration::from_secs(2)))?
            .ok_or_else(|| anyhow::anyhow!("Timeout in frame body"))
    };

    let seq = read_byte(port)?;

    let len_h = read_byte(port)?;
    let len_l = read_byte(port)?;
    let length = ((len_h as usize) << 8) | (len_l as usize);

    let mut payload = vec![0u8; length];
    if length > 0 {
        read_exact_deadline(port, &mut payload, deadline)?;
    }

    let crc_h = read_byte(port)?;
    let crc_l = read_byte(port)?;
    let rx_crc = ((crc_h as u16) << 8) | (crc_l as u16);

    // Verify CRC over SEQ + LEN + PAYLOAD
    let mut crc_data = vec![seq, len_h, len_l];
    crc_data.extend_from_slice(&payload);
    let calc_crc = crc16_ccitt(&crc_data);
    if calc_crc != rx_crc {
        bail!("CRC mismatch: calc=0x{calc_crc:04X} rx=0x{rx_crc:04X}");
    }

    Ok(FrameResult::Data(Frame { seq, payload }))
}

fn read_exact_deadline(port: &mut dyn serialport::SerialPort, buf: &mut [u8], deadline: Instant) -> Result<()> {
    let mut filled = 0;
    while filled < buf.len() {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            bail!("Timeout reading payload: got {filled}/{}", buf.len());
        }
        port.set_timeout(remaining.min(Duration::from_secs(2)))?;
        match port.read(&mut buf[filled..]) {
            Ok(n) => filled += n,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e.into()),
        }
    }
    Ok(())
}

/// Send a control byte, optionally with a sequence number.
pub fn send_control(port: &mut dyn serialport::SerialPort, ctrl: u8, seq: Option<u8>) -> Result<()> {
    port.write_all(&[ctrl])?;
    if let Some(s) = seq {
        port.write_all(&[s])?;
    }
    port.flush()?;
    Ok(())
}

/// Open and configure a serial port for SLIDE.
pub fn open_serial(port_name: &str, baud: u32) -> Result<Box<dyn serialport::SerialPort>> {
    let port = serialport::new(port_name, baud)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::Hardware)
        .timeout(Duration::from_secs(2))
        .open()?;
    Ok(port)
}

/// v0.2 §"Startup handshake": the sender transmits RDY first and the
/// receiver echoes it back. Returns Ok(true) once the echo arrives, or
/// Ok(false) if the peer cancelled or no echo arrived within `timeout`.
pub fn handshake_as_sender(
    port: &mut dyn serialport::SerialPort,
    timeout: Duration,
) -> Result<bool> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        port.write_all(&[CTRL_RDY])?;
        port.flush()?;
        thread::sleep(Duration::from_secs(1));

        match read_byte_timeout(port, Duration::from_secs(1))? {
            Some(CTRL_RDY) => {
                // v0.2.1 §1 wakeup bytes and RDY retries may still be queued
                thread::sleep(Duration::from_millis(50));
                port.clear(serialport::ClearBuffer::Input)?;
                return Ok(true);
            }
            Some(CTRL_CAN) => {
                echo_can_and_drain(port)?;
                return Ok(false);
            }
            // stray byte (wakeup signature, banner text) or timeout — retry
            _ => continue,
        }
    }
    Ok(false)
}

// ============================================================================
// v0.3 §2 — command-channel capability probe
// ============================================================================

/// VER is 4 ASCII digits, 'MMmm'.
pub const CMDSET_VER_LEN: usize = 4;
/// Major version this implementation speaks.
pub const CMDSET_MAJOR: u8 = 0;
/// Minor version this implementation speaks.
pub const CMDSET_MINOR: u8 = 1;

#[derive(Debug, Clone, PartialEq)]
pub enum ProbeOutcome {
    /// ENQ echoed with a well-formed VER.
    Supported { major: u8, minor: u8, ver_raw: Vec<u8> },
    /// Silence — peer is v0.2.1 or older.
    Unsupported,
    /// ENQ echoed, but VER was short or contained a non-digit.
    Malformed { ver_raw: Vec<u8>, reason: String },
    /// Peer sent CAN during the probe window (already echoed and drained).
    Cancelled,
}

#[derive(Debug, Clone)]
pub struct ProbeResult {
    pub outcome: ProbeOutcome,
    /// Non-ENQ bytes seen during the probe window.
    pub stray: Vec<u8>,
    pub attempts: u32,
}

impl ProbeResult {
    /// True only if command frames may actually be sent. v0.3 §2: an
    /// unknown major means opcode and record encodings may have changed
    /// underneath us, so we fall back exactly as for no echo.
    pub fn usable(&self) -> bool {
        matches!(self.outcome, ProbeOutcome::Supported { major, .. } if major == CMDSET_MAJOR)
    }
}

/// v0.3 §2: probe a peer for command-channel support.
///
/// Sends CTRL_ENQ at a frame boundary — where a v0.2.1 receiver expects a
/// file header or FIN. A v0.3 server echoes CTRL_ENQ followed by four ASCII
/// version digits. A v0.2.1 server skips the byte as stray and says nothing,
/// which is the entire point: the probe is inert, and the peer never leaves
/// its file loop, so the session survives intact.
///
/// MUST only be called at a frame boundary (v0.3 §2 "Where the client may
/// send it"): just after the RDY handshake, or after a completed file or
/// command exchange. Never mid-frame, and never between a control byte and
/// its sequence byte.
///
/// `settle` covers the v0.3 §2 "Timing" hazard: the Z80 flushes its receive
/// FIFO immediately after echoing RDY (slide.asm:876), so a probe fired
/// instantly can be swallowed. Retries cover it too.
pub fn probe_command_support(
    port: &mut dyn serialport::SerialPort,
    attempts: u32,
    echo_timeout: Duration,
    settle: Duration,
    debug: bool,
) -> Result<ProbeResult> {
    let mut stray: Vec<u8> = Vec::new();

    if !settle.is_zero() {
        thread::sleep(settle);
    }
    port.clear(serialport::ClearBuffer::Input)?;

    for attempt in 1..=attempts {
        if debug {
            eprintln!("    DEBUG probe attempt {attempt}/{attempts}: sending ENQ (0x05)");
        }
        port.write_all(&[CTRL_ENQ])?;
        port.flush()?;

        let deadline = Instant::now() + echo_timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            let Some(b) = read_byte_timeout(port, remaining)? else {
                break;
            };

            match b {
                CTRL_ENQ => {
                    if debug {
                        eprintln!("    DEBUG got ENQ echo, reading {CMDSET_VER_LEN} VER bytes");
                    }
                    let outcome = read_version(port, echo_timeout, debug)?;
                    return Ok(ProbeResult { outcome, stray, attempts: attempt });
                }
                CTRL_CAN => {
                    echo_can_and_drain(port)?;
                    return Ok(ProbeResult {
                        outcome: ProbeOutcome::Cancelled,
                        stray,
                        attempts: attempt,
                    });
                }
                other => {
                    // v0.2.1 peers send nothing at all here, so anything in
                    // this bucket is worth reporting.
                    stray.push(other);
                    if debug {
                        eprintln!("    DEBUG stray byte 0x{other:02X}");
                    }
                }
            }
        }
    }

    Ok(ProbeResult {
        outcome: ProbeOutcome::Unsupported,
        stray,
        attempts,
    })
}

/// Read and validate the 4-byte VER field following an ENQ echo.
fn read_version(
    port: &mut dyn serialport::SerialPort,
    timeout: Duration,
    debug: bool,
) -> Result<ProbeOutcome> {
    let mut ver: Vec<u8> = Vec::with_capacity(CMDSET_VER_LEN);
    let deadline = Instant::now() + timeout;

    while ver.len() < CMDSET_VER_LEN {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        match read_byte_timeout(port, remaining)? {
            Some(b) => ver.push(b),
            None => break,
        }
    }

    if ver.len() < CMDSET_VER_LEN {
        return Ok(ProbeOutcome::Malformed {
            reason: format!("VER truncated: got {} of {CMDSET_VER_LEN} bytes", ver.len()),
            ver_raw: ver,
        });
    }

    let Some((major, minor)) = parse_ver(&ver) else {
        return Ok(ProbeOutcome::Malformed {
            reason: format!("VER is not four ASCII digits: {}", hex_bytes(&ver)),
            ver_raw: ver,
        });
    };

    // v0.3 §2 idempotency: a retried probe draws more than one echo.
    thread::sleep(Duration::from_millis(50));
    port.clear(serialport::ClearBuffer::Input)?;

    if debug {
        eprintln!("    DEBUG VER = major {major}, minor {minor}");
    }
    Ok(ProbeOutcome::Supported { major, minor, ver_raw: ver })
}

/// Parse a v0.3 §2 VER field: four ASCII digits, 'MMmm'. Returns
/// (major, minor), or None if the field is the wrong length or not all
/// digits — which §2 says to treat as no command support, not as a guess.
pub(crate) fn parse_ver(ver: &[u8]) -> Option<(u8, u8)> {
    if ver.len() != CMDSET_VER_LEN || !ver.iter().all(|c| c.is_ascii_digit()) {
        return None;
    }
    Some((
        (ver[0] - b'0') * 10 + (ver[1] - b'0'),
        (ver[2] - b'0') * 10 + (ver[3] - b'0'),
    ))
}

fn hex_bytes(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16() {
        // Known CRC values
        assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
    }

    #[test]
    fn test_parse_ver() {
        // This spec's version.
        assert_eq!(parse_ver(b"0001"), Some((0, 1)));
        // Full range of both fields.
        assert_eq!(parse_ver(b"9999"), Some((99, 99)));
        assert_eq!(parse_ver(b"0300"), Some((3, 0)));
        // Wrong length — truncated or overlong.
        assert_eq!(parse_ver(b"001"), None);
        assert_eq!(parse_ver(b"00011"), None);
        assert_eq!(parse_ver(b""), None);
        // Non-digits, including control bytes that must never be accepted.
        assert_eq!(parse_ver(b"00\x01\x02"), None);
        assert_eq!(parse_ver(b"v0.3"), None);
    }

    #[test]
    fn test_probe_result_usable() {
        let supported = |major, minor| ProbeResult {
            outcome: ProbeOutcome::Supported {
                major,
                minor,
                ver_raw: b"0000".to_vec(),
            },
            stray: vec![],
            attempts: 1,
        };
        // Known major — usable, whatever the minor. v0.3 §2: the command
        // set is additive within a major.
        assert!(supported(CMDSET_MAJOR, 0).usable());
        assert!(supported(CMDSET_MAJOR, 99).usable());
        // Unknown major — encodings may have shifted, so commands MUST NOT
        // be sent even though the peer answered.
        assert!(!supported(CMDSET_MAJOR + 1, 1).usable());

        for outcome in [
            ProbeOutcome::Unsupported,
            ProbeOutcome::Cancelled,
            ProbeOutcome::Malformed {
                ver_raw: vec![],
                reason: String::new(),
            },
        ] {
            let r = ProbeResult { outcome, stray: vec![], attempts: 1 };
            assert!(!r.usable());
        }
    }

    #[test]
    fn test_enq_is_inert_to_v021() {
        // v0.3 §2 relies on 0x05 not colliding with any v0.2.1 control
        // byte — that is the whole reason a probe is safe to send blind.
        for ctrl in [SOF, CTRL_ACK, CTRL_NAK, CTRL_RDY, CTRL_FIN, CTRL_CAN] {
            assert_ne!(CTRL_ENQ, ctrl);
        }
        // VER digits must not collide either, so a late or duplicated echo
        // can never be misread as a control byte.
        for d in b'0'..=b'9' {
            for ctrl in [SOF, CTRL_ACK, CTRL_NAK, CTRL_RDY, CTRL_FIN, CTRL_CAN, CTRL_ENQ] {
                assert_ne!(d, ctrl);
            }
        }
    }

    #[test]
    fn test_build_frame_roundtrip() {
        let frame = build_frame(0x05, b"hello");
        assert_eq!(frame[0], SOF);
        assert_eq!(frame[1], 0x05);
        assert_eq!(frame[2], 0x00); // len_h
        assert_eq!(frame[3], 0x05); // len_l
        assert_eq!(&frame[4..9], b"hello");
        // Verify CRC
        let crc_data = &frame[1..9]; // seq + len + payload
        let crc = crc16_ccitt(crc_data);
        assert_eq!(frame[9], (crc >> 8) as u8);
        assert_eq!(frame[10], (crc & 0xFF) as u8);
    }
}
