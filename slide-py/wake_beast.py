#!/usr/bin/env python3
"""
wake_beast.py — recover a Beast whose UART RTS is deasserted.

After a power cycle the TL16C550's MCR is 0, so the Beast does not assert
RTS. The host therefore reads CTS low, and any host using hardware flow
control — BeasTTY, slide-send, slide probe — cannot transmit a single
byte. You cannot type the command that would fix it, because typing is
what is blocked.

`uart_init` in slide.asm is the only code on the Beast that sets
MCR_RTS | MCR_AFE, so running any SLIDE build asserts RTS and clears the
deadlock. This script does that with flow control bypassed, then cancels
out of receive mode so the Beast is left at the CP/M prompt.

Usage: wake_beast.py /dev/ttyUSB0 [--cmd "slide r"] [--baud 19200]
"""

import argparse
import sys
import time

import serial


def drain(ser, seconds):
    end = time.time() + seconds
    buf = bytearray()
    while time.time() < end:
        chunk = ser.read(512)
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def wake(port: str, cmd: str, baud: int) -> int:
    # rtscts=False is the whole point: we must be able to transmit while
    # the Beast is holding CTS low.
    ser = serial.Serial(port, baud, timeout=0.3, rtscts=False)
    ser.rts = True
    ser.dtr = True

    print(f"{port}: CTS={ser.cts} (False means the Beast is not asserting RTS)")
    if ser.cts:
        print("CTS is already high — nothing to recover.")
        ser.close()
        return 0

    ser.write(b'\r')
    ser.flush()
    print(f"prompt: {drain(ser, 1.0)!r}")

    print(f"running {cmd!r} to reach uart_init...")
    ser.write(cmd.encode('ascii') + b'\r')
    ser.flush()
    banner = drain(ser, 3.0)
    print(f"reply:  {banner!r}")

    if b'?' in banner and b'SLIDE' not in banner:
        print(f"\nCP/M could not find that command. Try another drive, "
              f"e.g. --cmd 'b:{cmd}'")
        ser.close()
        return 1

    print(f"CTS after uart_init = {ser.cts}")

    # Cancel out of receive mode so the Beast returns to the prompt
    # (v0.2.1 §2 — the Z80 echoes CAN and warm-boots).
    ser.write(bytes([0x18]))
    ser.flush()
    print(f"after CAN: {drain(ser, 2.5)!r}")

    ok = ser.cts
    print(f"\nFINAL CTS={ok} — "
          f"{'recovered, BeasTTY can transmit again' if ok else 'STILL BLOCKED'}")
    ser.close()
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port', help='Serial port (e.g. /dev/ttyUSB0)')
    ap.add_argument('--cmd', default='slide r',
                    help="SLIDE command to run; any build works, only "
                         "uart_init matters (default: 'slide r')")
    ap.add_argument('--baud', type=int, default=19200)
    args = ap.parse_args()
    sys.exit(wake(args.port, args.cmd, args.baud))


if __name__ == '__main__':
    main()
