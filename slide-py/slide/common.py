"""
SLIDE - Serial Line Inter-Device (file) Exchange
Shared protocol constants, CRC, frame building, and serial helpers.
"""

import serial
import struct
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

# Protocol constants
SOF       = 0x01
CTRL_ACK  = 0x06
CTRL_NAK  = 0x15
CTRL_RDY  = 0x11
CTRL_FIN  = 0x04   # end of session (was CTRL_EOT in v0.1)
CTRL_CAN  = 0x18
CTRL_ENQ  = 0x05   # v0.3 §2: "a command frame follows" / capability probe

WIN_SIZE   = 4
FRAME_SIZE = 1024

# v0.2.1 §1: Z80 emits this 7-byte signature on entering SLIDE mode,
# before any RDY/control/payload. PCs may use it as a liveness signal;
# they MUST tolerate it appearing on the wire as non-SOF / non-control bytes.
WAKEUP_SIG = bytes([0x1B, 0x5E, 0x53, 0x4C, 0x49, 0x44, 0x45])  # ESC ^ S L I D E


class CanReceived(Exception):
    """v0.2.1 §2: peer sent CTRL_CAN. The receiving helper has already
    echoed CAN back and drained the wire — caller should abort to idle."""
    pass


def _echo_can_and_drain(ser: serial.Serial):
    """v0.2.1 §2: echo CTRL_CAN back to the peer within ~500 ms, then drain
    so the wire is clean before either side starts the next session."""
    ser.write(bytes([CTRL_CAN]))
    ser.flush()
    time.sleep(0.05)
    ser.reset_input_buffer()


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16-CCITT (polynomial 0x1021, init 0xFFFF)."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def build_frame(seq: int, payload: bytes) -> bytes:
    """Build a complete wire frame with SOF, seq, length, payload, and CRC."""
    length = len(payload)
    crc_data = bytes([seq, length >> 8, length & 0xFF]) + payload
    crc = crc16_ccitt(crc_data)
    frame = bytes([SOF, seq, length >> 8, length & 0xFF]) + payload
    frame += struct.pack(">H", crc)
    return frame


def build_header_frame(filename: str, filesize: int) -> bytes:
    """Build header frame: null-terminated filename + 4-byte little-endian size."""
    name = Path(filename).name.upper()
    payload = name.encode('ascii') + b'\x00' + struct.pack('<I', filesize)
    return build_frame(0, payload)


def recv_control(ser: serial.Serial, timeout: float = 10.0) -> tuple:
    """
    Wait for a control byte from the remote side.
    Returns: (control_type, seq_or_none)
    Recognises ACK, NAK (with seq byte), RDY, CAN, FIN (no seq).

    v0.2.1: on CTRL_CAN, echoes CAN back and drains the wire before
    returning, so the caller can simply treat (CTRL_CAN, None) as a
    confirmed cancel and abort to idle.
    """
    ser.timeout = timeout
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("Timeout waiting for response")

        ctrl = b[0]
        if ctrl in (CTRL_ACK, CTRL_NAK):
            seq_byte = ser.read(1)
            if not seq_byte:
                raise TimeoutError("Timeout waiting for sequence byte")
            return (ctrl, seq_byte[0])
        elif ctrl == CTRL_CAN:
            _echo_can_and_drain(ser)
            return (CTRL_CAN, None)
        elif ctrl in (CTRL_RDY, CTRL_FIN):
            return (ctrl, None)
        # else: ignore spurious bytes (incl. wakeup signature bytes)


def open_serial(port: str, baud: int = 19200) -> serial.Serial:
    """Open a serial port configured for SLIDE protocol."""
    ser = serial.Serial(port, baud, timeout=2,
                        rtscts=True,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def send_start_command(ser: serial.Serial, cmd: str, debug: bool = False) -> None:
    """
    Type a command at the peer's CP/M prompt.

    On a MicroBeast the CP/M console *is* this UART, so writing the text
    here is exactly equivalent to typing it in a terminal. That matters for
    tools that own the serial port: there is otherwise no moment when both
    a terminal emulator and this program can hold the line, and the v0.2.1
    §1 wakeup contract needs the host asserting RTS *before* the Z80 enters
    SLIDE mode. Driving the prompt ourselves satisfies both.
    """
    # A bare CR first clears any half-typed line at the CCP prompt and
    # draws a fresh one; drop whatever that echoes before sending the
    # command proper.
    ser.write(b'\r')
    ser.flush()
    time.sleep(0.25)
    ser.reset_input_buffer()

    if debug:
        print(f"    DEBUG typing at CP/M prompt: {cmd!r}")
    ser.write(cmd.encode('ascii') + b'\r')
    ser.flush()


@dataclass
class HandshakeResult:
    connected: bool = False
    wakeup_seen: bool = False   # v0.2.1 §1 signature observed while waiting
    strays: int = 0             # non-control bytes skipped (banner, echo)


def handshake_as_sender(ser: serial.Serial, timeout: float = 60.0,
                        debug: bool = False) -> HandshakeResult:
    """
    v0.2 §"Startup handshake": the sender transmits RDY first and the
    receiver echoes it back.
    """
    result = HandshakeResult()
    wakeup_pos = 0
    deadline = time.time() + timeout
    while time.time() < deadline:
        ser.write(bytes([CTRL_RDY]))
        ser.flush()
        if debug:
            print("    DEBUG handshake: sent RDY, listening for 1s")

        # Drain everything that arrives in this window rather than testing a
        # single byte. The Z80 prints its banner and emits the v0.2.1 §1
        # wakeup signature before its RDY, so tens of stray bytes precede
        # the echo — one-byte-per-round would take tens of seconds.
        window = time.time() + 1.0
        while True:
            remaining = window - time.time()
            if remaining <= 0:
                break
            b = _read_byte(ser, remaining)
            if b is None:
                break
            if b == CTRL_RDY:
                # v0.2.1 §1 wakeup bytes and RDY retries may still be queued
                time.sleep(0.05)
                ser.reset_input_buffer()
                result.connected = True
                return result
            if b == CTRL_CAN:
                _echo_can_and_drain(ser)
                return result

            result.strays += 1
            # Log the byte before testing it, so the completion notice below
            # lands after the byte that completed the match rather than
            # appearing to jump a byte early.
            if debug:
                ch = chr(b) if 0x20 <= b < 0x7F else '.'
                print(f"    DEBUG handshake stray 0x{b:02X} '{ch}'")

            # Rolling match for the v0.2.1 §1 wakeup signature.
            if b == WAKEUP_SIG[wakeup_pos]:
                wakeup_pos += 1
                if wakeup_pos == len(WAKEUP_SIG):
                    result.wakeup_seen = True
                    wakeup_pos = 0
                    if debug:
                        print("    DEBUG ^ completed the wakeup signature (v0.2.1 §1)")
            else:
                wakeup_pos = 1 if b == WAKEUP_SIG[0] else 0
    return result


# ============================================================================
# v0.3 §2 — command-channel capability probe
# ============================================================================

CMDSET_VER_LEN = 4   # VER is 4 ASCII digits, 'MMmm'
CMDSET_MAJOR   = 0   # major version this implementation speaks
CMDSET_MINOR   = 2   # minor version this implementation speaks


class ProbeOutcome(str, Enum):
    SUPPORTED   = 'supported'    # ENQ echoed with a well-formed VER
    UNSUPPORTED = 'unsupported'  # silence — peer is v0.2.1 or older
    MALFORMED   = 'malformed'    # ENQ echoed, but VER was short or non-digit
    CANCELLED   = 'cancelled'    # peer sent CAN during the probe window


@dataclass
class ProbeResult:
    outcome: ProbeOutcome
    major: int = None
    minor: int = None
    ver_raw: bytes = b''
    stray: bytes = b''      # non-ENQ bytes seen during the probe window
    # Bytes already queued before the probe went out — residue from a
    # previous exchange. Captured rather than blindly cleared so it is
    # visible, but still removed before probing: an unconsumed `ACK 5` is
    # the bytes `06 05`, and that 0x05 would otherwise read as an echo.
    discarded: bytes = b''
    attempts: int = 0
    detail: str = ''

    @property
    def usable(self) -> bool:
        """
        True only if command frames may actually be sent. v0.3 §2: an
        unknown major means opcode and record encodings may have changed
        underneath us, so we fall back exactly as for no echo.
        """
        return self.outcome is ProbeOutcome.SUPPORTED and self.major == CMDSET_MAJOR


def _read_byte(ser: serial.Serial, timeout: float):
    """Read one byte with a timeout. Returns None on timeout."""
    ser.timeout = max(timeout, 0.0)
    b = ser.read(1)
    return b[0] if b else None


def drain_and_capture(ser: serial.Serial, quiet: float = 0.05,
                      cap: float = 0.5) -> bytes:
    """
    Read and return everything currently queued, stopping once the line has
    been quiet for `quiet` or `cap` has elapsed overall.

    Used in place of a blind input-buffer clear where the residue itself is
    diagnostic: leftovers from a previous exchange become visible instead of
    vanishing, while still being removed before the next exchange starts.
    """
    out = bytearray()
    deadline = time.time() + cap
    while time.time() < deadline:
        b = _read_byte(ser, quiet)
        if b is None:
            break
        out.append(b)
    return bytes(out)


def probe_command_support(ser: serial.Serial, attempts: int = 3,
                          echo_timeout: float = 0.5, settle: float = 0.1,
                          debug: bool = False) -> ProbeResult:
    """
    v0.3 §2: probe a peer for command-channel support.

    Sends CTRL_ENQ at a frame boundary — where a v0.2.1 receiver expects a
    file header or FIN. A v0.3 server echoes CTRL_ENQ followed by four
    ASCII version digits. A v0.2.1 server skips the byte as stray and says
    nothing, which is the entire point: the probe is inert, and the peer
    never leaves its file loop, so the session survives intact.

    MUST only be called at a frame boundary (v0.3 §2 "Where the client may
    send it"): just after the RDY handshake, or after a completed file or
    command exchange. Never mid-frame, and never between a control byte
    and its sequence byte.

    `settle` covers the v0.3 §2 "Timing" hazard: the Z80 flushes its
    receive FIFO immediately after echoing RDY (slide.asm:876), so a probe
    fired instantly can be swallowed. Retries cover it too.
    """
    saved_timeout = ser.timeout
    stray = bytearray()
    try:
        if settle > 0:
            time.sleep(settle)
        discarded = drain_and_capture(ser)
        if debug and discarded:
            print(f"    DEBUG discarded {len(discarded)} queued byte(s) before probing")

        for attempt in range(1, attempts + 1):
            if debug:
                print(f"    DEBUG probe attempt {attempt}/{attempts}: sending ENQ (0x05)")
            ser.write(bytes([CTRL_ENQ]))
            ser.flush()

            deadline = time.time() + echo_timeout
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                b = _read_byte(ser, remaining)
                if b is None:
                    break

                if b == CTRL_ENQ:
                    if debug:
                        print(f"    DEBUG got ENQ echo, reading {CMDSET_VER_LEN} VER bytes")
                    return _read_version(ser, echo_timeout, stray, discarded,
                                         attempt, debug)

                if b == CTRL_CAN:
                    _echo_can_and_drain(ser)
                    return ProbeResult(
                        outcome=ProbeOutcome.CANCELLED,
                        stray=bytes(stray),
                        discarded=discarded,
                        attempts=attempt,
                        detail="peer sent CAN during the probe window",
                    )

                # Anything else is stray. v0.2.1 peers send nothing at all
                # here, so bytes in this bucket are worth reporting.
                stray.append(b)
                if debug:
                    print(f"    DEBUG stray byte 0x{b:02X}")

        return ProbeResult(
            outcome=ProbeOutcome.UNSUPPORTED,
            stray=bytes(stray),
            discarded=discarded,
            attempts=attempts,
            detail="no ENQ echo — peer does not implement the v0.3 command channel",
        )
    finally:
        ser.timeout = saved_timeout


def _read_version(ser: serial.Serial, timeout: float, stray: bytearray,
                  discarded: bytes, attempt: int, debug: bool) -> ProbeResult:
    """Read and validate the 4-byte VER field following an ENQ echo."""
    ver = bytearray()
    deadline = time.time() + timeout
    while len(ver) < CMDSET_VER_LEN:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        b = _read_byte(ser, remaining)
        if b is None:
            break
        ver.append(b)

    if len(ver) < CMDSET_VER_LEN:
        return ProbeResult(
            outcome=ProbeOutcome.MALFORMED,
            ver_raw=bytes(ver),
            stray=bytes(stray),
            discarded=discarded,
            attempts=attempt,
            detail=f"VER truncated: got {len(ver)} of {CMDSET_VER_LEN} bytes",
        )

    if not all(0x30 <= c <= 0x39 for c in ver):
        return ProbeResult(
            outcome=ProbeOutcome.MALFORMED,
            ver_raw=bytes(ver),
            stray=bytes(stray),
            discarded=discarded,
            attempts=attempt,
            detail=f"VER is not four ASCII digits: {ver.hex(' ')}",
        )

    # v0.3 §2 idempotency: a retried probe draws more than one echo.
    time.sleep(0.05)
    ser.reset_input_buffer()

    text = ver.decode('ascii')
    return ProbeResult(
        outcome=ProbeOutcome.SUPPORTED,
        major=int(text[:2]),
        minor=int(text[2:]),
        ver_raw=bytes(ver),
        stray=bytes(stray),
        discarded=discarded,
        attempts=attempt,
    )


# ============================================================================
# v0.3 §3/§4 — command frames and reply streams
# ============================================================================

CMD_NOP  = 0x00   # no-op; completes a probe-only exchange
CMD_VOLS = 0x01
CMD_DIR  = 0x02
CMD_DEL  = 0x03
CMD_REN  = 0x04

_STATUS_NAMES = {
    0x00: "ST_OK",
    0x01: "ST_OPCODE — unknown or unsupported opcode",
    0x02: "ST_OPERAND — missing or malformed operand",
    0x03: "ST_NODRIVE — drive not present or not selectable",
    0x04: "ST_IO — BDOS/BIOS error during enumeration",
    0x05: "ST_BUSY — server cannot service the request now",
    0x06: "ST_NOFILE — nothing matched",
    0x07: "ST_EXISTS — destination already exists",
    0x08: "ST_RO — file is read-only",
}


def status_name(status: int) -> str:
    """Human-readable name for a v0.3 §4 status code."""
    return _STATUS_NAMES.get(status, "unknown status")


@dataclass
class CommandReply:
    status: int
    records: bytes = b''


def send_command(ser: serial.Serial, opcode: int, operands: bytes = b'',
                 debug: bool = False) -> CommandReply:
    """
    Send a v0.3 §3 command frame and read the §4 reply stream.

    The caller MUST already have had a CTRL_ENQ echoed (§2) — the server is
    in command mode and this frame is what it is waiting for. Sending a
    command frame without that echo is forbidden by §6.
    """
    from slide.recv import recv_frame, send_control, _FinReceived

    payload = bytes([opcode]) + operands
    frame = build_frame(0, payload)
    if debug:
        print(f"    DEBUG command frame: {frame.hex(' ')}")
    ser.write(frame)
    ser.flush()

    # §4: the server ACKs the command frame before replying.
    ctrl, seq = recv_control(ser, timeout=5.0)
    if ctrl == CTRL_NAK:
        raise ValueError(f"server NAKed the command frame (seq {seq})")
    if ctrl == CTRL_CAN:
        raise CanReceived("server cancelled instead of accepting the command")
    if ctrl != CTRL_ACK or seq != 0:
        raise ValueError(f"expected ACK 0 for the command frame, got 0x{ctrl:02X} seq={seq}")

    # §4: status frame (seq 1), record frames, zero-length terminator.
    body = bytearray()
    expected_seq = 1
    while True:
        try:
            seq, chunk = recv_frame(ser, timeout=10.0)
        except _FinReceived:
            raise ValueError("unexpected FIN during the reply stream")

        if not chunk:
            if debug:
                print(f"    DEBUG reply terminator seq={seq}")
            send_control(ser, CTRL_ACK, seq)
            break
        if seq != expected_seq:
            send_control(ser, CTRL_NAK, expected_seq)
            continue
        if debug:
            print(f"    DEBUG reply frame seq={seq} len={len(chunk)}")
        body.extend(chunk)
        expected_seq = (expected_seq + 1) & 0xFF
        # ACK every WIN_SIZE frames, as for file data.
        if (expected_seq - 1) & (WIN_SIZE - 1) == 0:
            send_control(ser, CTRL_ACK, seq)

    if not body:
        raise ValueError("reply stream ended without a status frame")
    return CommandReply(status=body[0], records=bytes(body[1:]))
