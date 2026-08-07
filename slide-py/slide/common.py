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


def handshake_as_sender(ser: serial.Serial, timeout: float = 60.0) -> bool:
    """
    v0.2 §"Startup handshake": the sender transmits RDY first and the
    receiver echoes it back. Returns True once the echo arrives.

    Returns False if the peer sent CAN (already echoed and drained) or if
    no echo arrived within `timeout`.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        ser.write(bytes([CTRL_RDY]))
        ser.flush()
        time.sleep(1.0)

        ser.timeout = 1.0
        b = ser.read(1)
        if not b:
            continue
        if b[0] == CTRL_RDY:
            # v0.2.1 §1 wakeup bytes and RDY retries may still be queued
            time.sleep(0.05)
            ser.reset_input_buffer()
            return True
        if b[0] == CTRL_CAN:
            _echo_can_and_drain(ser)
            return False
        # else: stray byte (wakeup signature, banner text) — keep trying
    return False


# ============================================================================
# v0.3 §2 — command-channel capability probe
# ============================================================================

CMDSET_VER_LEN = 4   # VER is 4 ASCII digits, 'MMmm'
CMDSET_MAJOR   = 0   # major version this implementation speaks
CMDSET_MINOR   = 1   # minor version this implementation speaks


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
        ser.reset_input_buffer()

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
                    return _read_version(ser, echo_timeout, stray, attempt, debug)

                if b == CTRL_CAN:
                    _echo_can_and_drain(ser)
                    return ProbeResult(
                        outcome=ProbeOutcome.CANCELLED,
                        stray=bytes(stray),
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
            attempts=attempts,
            detail="no ENQ echo — peer does not implement the v0.3 command channel",
        )
    finally:
        ser.timeout = saved_timeout


def _read_version(ser: serial.Serial, timeout: float, stray: bytearray,
                  attempt: int, debug: bool) -> ProbeResult:
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
            attempts=attempt,
            detail=f"VER truncated: got {len(ver)} of {CMDSET_VER_LEN} bytes",
        )

    if not all(0x30 <= c <= 0x39 for c in ver):
        return ProbeResult(
            outcome=ProbeOutcome.MALFORMED,
            ver_raw=bytes(ver),
            stray=bytes(stray),
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
        attempts=attempt,
    )
