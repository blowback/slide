#!/usr/bin/env python3
"""
SLIDE - Serial Line Inter-Device (file) Exchange
PC-side sender for custom Z80 serial transfer protocol.

Usage: slide-send <serial_port> <filename> [--baud 19200]

Protocol:
  Frame: [SOF=0x01] [SEQ] [LEN_H] [LEN_L] [PAYLOAD...] [CRC_H] [CRC_L]
  Control (Z80->PC): [ACK 0x06] [SEQ] | [NAK 0x15] [SEQ] | [RDY 0x11]
  Session: header frame -> streaming data frames -> zero-length end frame
"""

import sys
import os
import time
import argparse

from slide.common import (
    SOF, CTRL_ACK, CTRL_NAK, CTRL_RDY, CTRL_FIN, CTRL_CAN,
    WIN_SIZE, FRAME_SIZE,
    crc16_ccitt, build_frame, build_header_frame, recv_control, open_serial,
)


def send_file(ser, filename: str, debug: bool = False):
    """Send a single file over an already-open serial connection."""

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
    print(f"  File: {filename}")
    print(f"  Size: {filesize} bytes ({total_frames} frames)")

    # --- Send header frame ---
    print("  Sending header frame...")
    header = build_header_frame(filename, filesize)
    if debug:
        print(f"    DEBUG header ({len(header)} bytes): {header.hex(' ')}")
    ser.write(header)
    ser.flush()

    ctrl, seq_ack = recv_control(ser, timeout=60.0)
    if debug:
        print(f"    DEBUG response: ctrl=0x{ctrl:02X} seq={seq_ack}")
    if ctrl == CTRL_CAN:
        print(f"  ERROR: Z80 rejected file — check disk/drive.")
        return False
    if ctrl != CTRL_ACK:
        print(f"  ERROR: Expected ACK for header, got {ctrl:#04x}")
        return False

    print("  Header acknowledged. Streaming data...\n")

    # Give Z80 time to create the file on disk before we start streaming
    time.sleep(0.5)
    ser.reset_input_buffer()

    # --- Stream data frames with sliding window ---
    send_idx = 0
    eof_seq = (frames[-1][0] + 1) & 0xFF if frames else 1
    eof_sent = False
    start_time = time.time()

    while send_idx < total_frames:
        window_start = send_idx
        window_end = min(send_idx + WIN_SIZE, total_frames)

        for i in range(window_start, window_end):
            seq_num, payload = frames[i]
            frame = build_frame(seq_num, payload)
            if debug:
                print(f"    DEBUG sending frame seq={seq_num} len={len(payload)} ({len(frame)} bytes on wire)")
            ser.write(frame)

        # If this is the last window, append EOF so Z80 doesn't wait
        if window_end >= total_frames and not eof_sent:
            eof_frame = build_frame(eof_seq, b'')
            if debug:
                print(f"    DEBUG sending EOF frame seq={eof_seq}")
            ser.write(eof_frame)
            eof_sent = True

        ser.flush()

        # Wait for response (skip RDY signals, wait for ACK/NAK/CAN)
        retry = False
        while True:
            try:
                ctrl, resp_seq = recv_control(ser, timeout=10.0)
            except TimeoutError:
                print(f"\n  Timeout at frame {send_idx}. Retrying window...")
                retry = True
                break
            if ctrl == CTRL_CAN:
                print(f"\n\n  Z80 reported disk error — transfer aborted.")
                return False
            if ctrl != CTRL_RDY:
                break
            if debug:
                print(f"    DEBUG got RDY (Z80 flushed to disk)")
        if retry:
            continue

        if debug:
            ctrl_name = {CTRL_ACK: 'ACK', CTRL_NAK: 'NAK'}.get(ctrl, f'0x{ctrl:02X}')
            print(f"    DEBUG got {ctrl_name} seq={resp_seq}")

        if ctrl == CTRL_ACK:
            acked_seq = resp_seq
            while send_idx < total_frames:
                fseq = frames[send_idx][0]
                send_idx += 1
                if fseq == acked_seq:
                    break
            if acked_seq == eof_seq:
                send_idx = total_frames

            pct = send_idx * 100 // total_frames
            bar = '#' * (pct // 2) + '-' * (50 - pct // 2)
            print(f"\r  [{bar}] {pct}% ({send_idx}/{total_frames})", end='')

        elif ctrl == CTRL_NAK:
            eof_sent = False
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

    # Wait for final EOF ACK
    try:
        ctrl, _ = recv_control(ser, timeout=2.0)
    except TimeoutError:
        pass

    elapsed = time.time() - start_time
    throughput = filesize / elapsed if elapsed > 0 else 0

    print(f"\n\n  Transfer complete!")
    print(f"  {filesize} bytes in {elapsed:.1f}s ({throughput:.0f} bytes/sec)")
    baud = ser.baudrate
    print(f"  Link efficiency: {throughput / (baud / 10) * 100:.0f}%")
    return True


def send_session(port: str, files: list, baud: int = 19200, debug: bool = False):
    """Send one or more files to the Z80 using SLIDE v0.2 session protocol."""

    print(f"SLIDE v0.2 - Serial Line Inter-Device Exchange (send)")
    print(f"  Port: {port} @ {baud} baud")
    print(f"  Files: {len(files)}")
    print()

    ser = open_serial(port, baud)

    # --- Handshake: sender sends RDY first, waits for receiver's RDY echo ---
    print("Waiting for Z80 (start SLIDE R on Z80 now)...")
    ser.timeout = 60
    while True:
        ser.write(bytes([CTRL_RDY]))
        ser.flush()
        time.sleep(1.0)
        # Check if Z80 echoed RDY back
        ser.timeout = 1.0
        b = ser.read(1)
        if b and b[0] == CTRL_RDY:
            break
    print("Z80 ready.")
    time.sleep(0.05)
    ser.reset_input_buffer()

    # --- Send each file ---
    for i, filename in enumerate(files):
        print(f"\n--- File {i+1}/{len(files)}: {filename} ---")
        ok = send_file(ser, filename, debug)
        if not ok:
            print("Session aborted.")
            ser.close()
            return

    # --- FIN exchange ---
    if debug:
        print("  DEBUG sending FIN")
    ser.write(bytes([CTRL_FIN]))
    ser.flush()
    # Wait for FIN echo
    try:
        ctrl, _ = recv_control(ser, timeout=5.0)
        if debug:
            print(f"  DEBUG got FIN echo: ctrl=0x{ctrl:02X}")
    except TimeoutError:
        pass

    print(f"\nSession complete — {len(files)} file(s) sent.")
    ser.close()


def main():
    parser = argparse.ArgumentParser(description='SLIDE - Send files to Z80/CP/M')
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('files', nargs='+', help='File(s) to send')
    parser.add_argument('--baud', type=int, default=19200, help='Baud rate (default: 19200)')
    parser.add_argument('--debug', action='store_true', help='Show wire-level debug output')
    args = parser.parse_args()

    for f in args.files:
        if not os.path.exists(f):
            print(f"Error: file '{f}' not found")
            sys.exit(1)

    send_session(args.port, args.files, args.baud, args.debug)


if __name__ == '__main__':
    main()
