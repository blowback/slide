"""
SLIDE - Serial Line Inter-Device (file) Exchange
Shared protocol constants, CRC, frame building, and serial helpers.
"""

import serial
import struct
from pathlib import Path

# Protocol constants
SOF       = 0x01
CTRL_ACK  = 0x06
CTRL_NAK  = 0x15
CTRL_RDY  = 0x11
CTRL_FIN  = 0x04   # end of session (was CTRL_EOT in v0.1)
CTRL_CAN  = 0x18

WIN_SIZE   = 4
FRAME_SIZE = 1024


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
        elif ctrl in (CTRL_RDY, CTRL_CAN, CTRL_FIN):
            return (ctrl, None)
        # else: ignore spurious bytes


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
