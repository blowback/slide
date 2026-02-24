#!/usr/bin/env python3
"""
SLIDE - Serial Line Inter-Device (file) Exchange
PC-side sender for custom Z80 serial transfer protocol.

Usage: python slide_send.py <serial_port> <filename> [--baud 19200]

Protocol:
  Frame: [SOF=0x01] [SEQ] [LEN_H] [LEN_L] [PAYLOAD...] [CRC_H] [CRC_L]
  Control (Z80->PC): [ACK 0x06] [SEQ] | [NAK 0x15] [SEQ] | [RDY 0x11]
  Session: header frame -> streaming data frames -> zero-length end frame
"""

import serial
import struct
import sys
import os
import time
import argparse
from pathlib import Path

# Protocol constants
SOF       = 0x01
CTRL_ACK  = 0x06
CTRL_NAK  = 0x15
CTRL_RDY  = 0x11
CTRL_EOT  = 0x04

WIN_SIZE   = 4
FRAME_SIZE = 1024

# CRC-16-CCITT (polynomial 0x1021, init 0xFFFF)
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
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
    # CRC covers seq + len_h + len_l + payload
    crc_data = bytes([seq, length >> 8, length & 0xFF]) + payload
    crc = crc16_ccitt(crc_data)
    frame = bytes([SOF, seq, length >> 8, length & 0xFF]) + payload
    frame += struct.pack(">H", crc)  # CRC high byte first
    return frame


def build_header_frame(filename: str, filesize: int) -> bytes:
    """Build header frame: null-terminated filename + 4-byte little-endian size."""
    # Convert filename to 8.3 uppercase
    name = Path(filename).name.upper()
    payload = name.encode('ascii') + b'\x00' + struct.pack('<I', filesize)
    return build_frame(0, payload)


def recv_control(ser: serial.Serial, timeout: float = 10.0) -> tuple:
    """
    Wait for a control byte from Z80.
    Returns: (control_type, seq_or_none)
    """
    ser.timeout = timeout
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("Timeout waiting for Z80 response")

        ctrl = b[0]
        if ctrl in (CTRL_ACK, CTRL_NAK):
            seq_byte = ser.read(1)
            if not seq_byte:
                raise TimeoutError("Timeout waiting for sequence byte")
            return (ctrl, seq_byte[0])
        elif ctrl == CTRL_RDY:
            return (ctrl, None)
        # else: ignore spurious bytes


def send_file(port: str, filename: str, baud: int = 19200, debug: bool = False):
    """Send a file to the Z80 using the SLIDE protocol."""

    filesize = os.path.getsize(filename)
    with open(filename, 'rb') as f:
        file_data = f.read()

    # Pad to 128-byte boundary for CP/M
    if len(file_data) % 128:
        file_data += b'\x1A' * (128 - (len(file_data) % 128))

    # Split into frames
    frames = []
    offset = 0
    seq = 1  # seq 0 is the header
    while offset < len(file_data):
        chunk = file_data[offset:offset + FRAME_SIZE]
        frames.append((seq & 0xFF, chunk))
        seq += 1
        offset += FRAME_SIZE

    total_frames = len(frames)
    print(f"SLIDE - Serial Line Inter-Device Exchange")
    print(f"  File: {filename}")
    print(f"  Size: {filesize} bytes ({total_frames} frames)")
    print(f"  Port: {port} @ {baud} baud")
    print()

    ser = serial.Serial(port, baud, timeout=2,
                        rtscts=True,        # hardware flow control
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE)

    # Flush any garbage
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # --- Wait for Z80 to signal ready ---
    print("Waiting for Z80 (start SLIDE.COM on Z80 now)...")
    ser.timeout = 60  # generous timeout for human to start Z80 side
    while True:
        b = ser.read(1)
        if not b:
            print("ERROR: Timeout waiting for Z80 ready signal")
            ser.close()
            return
        if b[0] == CTRL_RDY:
            break
        # ignore any other bytes (e.g. CP/M boot noise)
    print("Z80 ready.")
    time.sleep(0.05)  # brief settle

    # --- Send header frame ---
    print("Sending header frame...")
    header = build_header_frame(filename, filesize)
    if debug:
        print(f"  DEBUG header ({len(header)} bytes): {header.hex(' ')}")
    ser.write(header)
    ser.flush()

    ctrl, seq_ack = recv_control(ser)
    if debug:
        print(f"  DEBUG response: ctrl=0x{ctrl:02X} seq={seq_ack}")
    if ctrl != CTRL_ACK:
        print(f"ERROR: Expected ACK for header, got {ctrl:#04x}")
        ser.close()
        return

    print("Header acknowledged. Streaming data...\n")

    # Give Z80 time to create the file on disk before we start streaming
    time.sleep(0.5)
    ser.reset_input_buffer()  # clear any noise during file creation

    # --- Stream data frames with sliding window ---
    send_idx = 0        # next frame index to send
    eof_seq = (frames[-1][0] + 1) & 0xFF if frames else 1
    eof_sent = False
    start_time = time.time()

    while send_idx < total_frames:
        # Send up to WIN_SIZE frames
        window_start = send_idx
        window_end = min(send_idx + WIN_SIZE, total_frames)

        for i in range(window_start, window_end):
            seq_num, payload = frames[i]
            frame = build_frame(seq_num, payload)
            if debug:
                print(f"  DEBUG sending frame seq={seq_num} len={len(payload)} ({len(frame)} bytes on wire)")
            ser.write(frame)

        # If this is the last window, append EOF so Z80 doesn't wait
        if window_end >= total_frames and not eof_sent:
            eof_frame = build_frame(eof_seq, b'')
            if debug:
                print(f"  DEBUG sending EOF frame seq={eof_seq}")
            ser.write(eof_frame)
            eof_sent = True

        ser.flush()

        # Wait for response (skip RDY signals, wait for ACK/NAK)
        retry = False
        while True:
            try:
                ctrl, resp_seq = recv_control(ser, timeout=10.0)
            except TimeoutError:
                print(f"\nTimeout at frame {send_idx}. Retrying window...")
                retry = True
                break
            if ctrl != CTRL_RDY:
                break
            if debug:
                print(f"  DEBUG got RDY (Z80 flushed to disk)")
        if retry:
            continue

        if debug:
            ctrl_name = {CTRL_ACK: 'ACK', CTRL_NAK: 'NAK'}.get(ctrl, f'0x{ctrl:02X}')
            print(f"  DEBUG got {ctrl_name} seq={resp_seq}")

        if ctrl == CTRL_ACK:
            # ACK received - advance window
            acked_seq = resp_seq
            while send_idx < total_frames:
                fseq = frames[send_idx][0]
                send_idx += 1
                if fseq == acked_seq:
                    break
            # If ACK was for EOF seq, all data is implicitly acked
            if acked_seq == eof_seq:
                send_idx = total_frames

            # Progress bar
            pct = send_idx * 100 // total_frames
            bar = '#' * (pct // 2) + '-' * (50 - pct // 2)
            print(f"\r  [{bar}] {pct}% ({send_idx}/{total_frames})", end='')

        elif ctrl == CTRL_NAK:
            # NAK - retransmit from requested sequence
            eof_sent = False  # re-append EOF on retransmit of last window
            print(f"\n  NAK received for seq {resp_seq}, retransmitting...")
            for i, (fseq, _) in enumerate(frames):
                if fseq == resp_seq:
                    send_idx = i
                    break


    # --- Send end-of-transfer (if not already sent with last window) ---
    if not eof_sent:
        eof_frame = build_frame(eof_seq, b'')
        ser.write(eof_frame)
        ser.flush()

    # Wait for final EOF ACK (full last window: data ACK already consumed,
    # EOF ACK still pending. Partial last window: EOF ACK already consumed.)
    try:
        ctrl, _ = recv_control(ser, timeout=2.0)
    except TimeoutError:
        pass

    elapsed = time.time() - start_time
    throughput = filesize / elapsed if elapsed > 0 else 0

    print(f"\n\nTransfer complete!")
    print(f"  {filesize} bytes in {elapsed:.1f}s ({throughput:.0f} bytes/sec)")
    print(f"  Link efficiency: {throughput / (baud / 10) * 100:.0f}%")

    ser.close()


def main():
    parser = argparse.ArgumentParser(description='SLIDE - Send files to Z80/CP/M')
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('filename', help='File to send')
    parser.add_argument('--baud', type=int, default=19200, help='Baud rate (default: 19200)')
    parser.add_argument('--debug', action='store_true', help='Show wire-level debug output')
    args = parser.parse_args()
    args = parser.parse_args()

    if not os.path.exists(args.filename):
        print(f"Error: file '{args.filename}' not found")
        sys.exit(1)

    send_file(args.port, args.filename, args.baud, args.debug)


if __name__ == '__main__':
    main()
