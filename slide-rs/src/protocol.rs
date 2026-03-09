use anyhow::{bail, Result};
use std::time::{Duration, Instant};

// Protocol constants
pub const SOF: u8 = 0x01;
pub const CTRL_ACK: u8 = 0x06;
pub const CTRL_NAK: u8 = 0x15;
pub const CTRL_RDY: u8 = 0x11;
pub const CTRL_FIN: u8 = 0x04;
pub const CTRL_CAN: u8 = 0x18;

pub const WIN_SIZE: usize = 4;
pub const FRAME_SIZE: usize = 1024;

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
fn read_byte_timeout(port: &mut dyn serialport::SerialPort, timeout: Duration) -> Result<Option<u8>> {
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
            CTRL_CAN => return Ok(Control::Can),
            CTRL_FIN => return Ok(Control::Fin),
            _ => continue, // ignore spurious bytes
        }
    }
}

/// Received frame data.
pub struct Frame {
    pub seq: u8,
    pub payload: Vec<u8>,
}

/// Result of recv_frame: either a data frame or FIN.
pub enum FrameResult {
    Data(Frame),
    Fin,
}

/// Receive a SLIDE frame from the serial port.
pub fn recv_frame(port: &mut dyn serialport::SerialPort, timeout: Duration) -> Result<FrameResult> {
    let deadline = Instant::now() + timeout;

    // Wait for SOF or FIN
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16() {
        // Known CRC values
        assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
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
