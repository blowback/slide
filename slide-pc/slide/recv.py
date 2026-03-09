#!/usr/bin/env python3
"""
SLIDE - Serial Line Inter-Device (file) Exchange
PC-side receiver: accepts files sent from Z80/CP/M.

Usage: slide-recv <serial_port> [--baud 19200] [--output-dir .]
"""

import struct
import sys
import os
import time
import argparse
from pathlib import Path

from slide.common import (
    SOF, CTRL_ACK, CTRL_NAK, CTRL_RDY, CTRL_FIN, CTRL_CAN,
    WIN_SIZE, FRAME_SIZE,
    crc16_ccitt, open_serial,
)


def recv_frame(ser, timeout: float = 10.0):
    """
    Receive a SLIDE frame from the serial port.
    Returns: (seq, payload) on success
    Raises: TimeoutError, ValueError (CRC mismatch)
    """
    ser.timeout = timeout

    # Wait for SOF
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("Timeout waiting for SOF")
        if b[0] == SOF:
            break
        if b[0] == CTRL_FIN:
            raise _FinReceived()

    return _recv_frame_after_sof(ser)


class _FinReceived(Exception):
    """Raised when FIN is received instead of SOF."""
    pass


def _recv_frame_after_sof(ser):
    """Receive the rest of a frame after SOF has been consumed."""
    # SEQ
    b = ser.read(1)
    if not b:
        raise TimeoutError("Timeout waiting for SEQ")
    seq = b[0]

    # LEN_H, LEN_L
    b = ser.read(2)
    if len(b) < 2:
        raise TimeoutError("Timeout waiting for LEN")
    length = (b[0] << 8) | b[1]

    # Payload
    payload = b''
    if length > 0:
        payload = ser.read(length)
        if len(payload) < length:
            raise TimeoutError(f"Timeout in payload: got {len(payload)}/{length}")

    # CRC_H, CRC_L
    b = ser.read(2)
    if len(b) < 2:
        raise TimeoutError("Timeout waiting for CRC")
    rx_crc = (b[0] << 8) | b[1]

    # Verify CRC over SEQ + LEN + PAYLOAD
    crc_data = bytes([seq, length >> 8, length & 0xFF]) + payload
    calc_crc = crc16_ccitt(crc_data)
    if calc_crc != rx_crc:
        raise ValueError(f"CRC mismatch: calc=0x{calc_crc:04X} rx=0x{rx_crc:04X}")

    return (seq, payload)


def send_control(ser, ctrl: int, seq: int = None):
    """Send a control byte, optionally followed by a sequence number."""
    ser.write(bytes([ctrl]))
    if seq is not None:
        ser.write(bytes([seq]))
    ser.flush()


def recv_one_file(ser, output_dir: str, debug: bool = False) -> bool:
    """
    Receive a single file (header already expected as next frame).
    Returns True on success, False on error.
    """
    # --- Receive header frame (seq 0) ---
    try:
        seq, payload = recv_frame(ser, timeout=30.0)
    except _FinReceived:
        return None  # session done
    except (TimeoutError, ValueError) as e:
        print(f"  Error receiving header: {e}")
        return False

    if debug:
        print(f"  DEBUG header seq={seq} len={len(payload)}: {payload.hex(' ')}")

    # Parse header: null-terminated filename + 4-byte LE size
    null_idx = payload.index(0)
    filename = payload[:null_idx].decode('ascii')
    filesize = struct.unpack('<I', payload[null_idx+1:null_idx+5])[0]

    filepath = os.path.join(output_dir, filename)
    print(f"  Receiving: {filename} ({filesize} bytes)")

    # ACK the header
    send_control(ser, CTRL_ACK, 0)

    # --- Receive data frames ---
    expected_seq = 1
    received_data = bytearray()
    retry_count = 0
    max_retries = 15

    while True:
        try:
            seq, payload = recv_frame(ser, timeout=10.0)
        except _FinReceived:
            print("  Unexpected FIN during file transfer")
            return False
        except TimeoutError as e:
            retry_count += 1
            if retry_count >= max_retries:
                print(f"\n  Aborted: too many retries")
                return False
            if debug:
                print(f"  DEBUG timeout, NAK seq={expected_seq}")
            send_control(ser, CTRL_NAK, expected_seq)
            continue
        except ValueError as e:
            retry_count += 1
            if retry_count >= max_retries:
                print(f"\n  Aborted: too many CRC errors")
                return False
            if debug:
                print(f"  DEBUG CRC error: {e}, NAK seq={expected_seq}")
            send_control(ser, CTRL_NAK, expected_seq)
            continue

        # Reset retry counter on successful frame
        retry_count = 0

        # Zero-length = end of file
        if len(payload) == 0:
            if debug:
                print(f"  DEBUG EOF frame seq={seq}")
            send_control(ser, CTRL_ACK, expected_seq)
            break

        # Check sequence
        if seq != expected_seq:
            if debug:
                print(f"  DEBUG seq mismatch: got {seq} expected {expected_seq}")
            send_control(ser, CTRL_NAK, expected_seq)
            continue

        received_data.extend(payload)
        expected_seq = (expected_seq + 1) & 0xFF

        if debug:
            print(f"  DEBUG frame seq={seq} len={len(payload)} total={len(received_data)}")

        # ACK every WIN_SIZE frames
        if (expected_seq - 1) & (WIN_SIZE - 1) == 0:
            send_control(ser, CTRL_ACK, seq)
            if debug:
                print(f"  DEBUG ACK seq={seq}")

        # Progress
        pct = min(len(received_data) * 100 // filesize, 100) if filesize > 0 else 100
        bar = '#' * (pct // 2) + '-' * (50 - pct // 2)
        print(f"\r  [{bar}] {pct}%", end='')

    # Truncate to actual file size (remove CP/M padding)
    file_data = bytes(received_data[:filesize])

    with open(filepath, 'wb') as f:
        f.write(file_data)

    print(f"\n  Saved: {filepath} ({len(file_data)} bytes)")
    return True


def recv_session(port: str, baud: int = 19200, output_dir: str = '.', debug: bool = False):
    """Receive one or more files from Z80 using SLIDE v0.2 session protocol."""

    print(f"SLIDE v0.2 - Serial Line Inter-Device Exchange (receive)")
    print(f"  Port: {port} @ {baud} baud")
    print(f"  Output: {output_dir}")
    print()

    os.makedirs(output_dir, exist_ok=True)
    ser = open_serial(port, baud)

    # --- Handshake: wait for sender's RDY, echo RDY back ---
    print("Waiting for Z80 sender (start SLIDE S <file> on Z80 now)...")
    ser.timeout = 60
    while True:
        b = ser.read(1)
        if not b:
            print("ERROR: Timeout waiting for Z80 ready signal")
            ser.close()
            return
        if b[0] == CTRL_RDY:
            break

    # Echo RDY back
    ser.write(bytes([CTRL_RDY]))
    ser.flush()
    print("Z80 connected.")
    time.sleep(0.05)
    ser.reset_input_buffer()

    # --- Receive files until FIN ---
    file_count = 0
    start_time = time.time()

    while True:
        print(f"\n--- Waiting for file {file_count + 1}... ---")
        result = recv_one_file(ser, output_dir, debug)
        if result is None:
            # FIN received — session done
            send_control(ser, CTRL_FIN)
            if debug:
                print("  DEBUG sent FIN echo")
            break
        elif result:
            file_count += 1
        else:
            print("  File transfer failed, aborting session.")
            break

    elapsed = time.time() - start_time
    print(f"\nSession complete — {file_count} file(s) received in {elapsed:.1f}s.")
    ser.close()


def main():
    parser = argparse.ArgumentParser(description='SLIDE - Receive files from Z80/CP/M')
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--baud', type=int, default=19200, help='Baud rate (default: 19200)')
    parser.add_argument('--output-dir', default='.', help='Directory for received files (default: .)')
    parser.add_argument('--debug', action='store_true', help='Show wire-level debug output')
    args = parser.parse_args()

    recv_session(args.port, args.baud, args.output_dir, args.debug)


if __name__ == '__main__':
    main()
