#!/usr/bin/env python3
"""
beast_term.py — a dumb serial terminal for a MicroBeast.

A standalone console for when BeasTTY is unavailable. Deliberately opens
with hardware flow control OFF, so it still works against a Beast that is
not asserting RTS — the power-cycle deadlock that locks out every
flow-controlled host (see wake_beast.py). Nothing here gates transmission
on CTS, so you can always type.

  Usage:  beast_term.py /dev/ttyUSB2 [--baud 19200]
  Quit:   Ctrl-]

Local echo is off: CP/M echoes your keystrokes itself. Enter sends CR
(0x0D), which is what the CCP expects.
"""

import argparse
import os
import select
import sys
import termios
import tty

import serial

QUIT_KEY = 0x1D  # Ctrl-]


def run(port_name: str, baud: int) -> int:
    try:
        ser = serial.Serial(port_name, baud, timeout=0, rtscts=False)
    except serial.SerialException as e:
        print(f"Could not open {port_name}: {e}", file=sys.stderr)
        print("If Chrome/BeasTTY still holds it, close that tab first "
              "(check Chrome's Task manager).", file=sys.stderr)
        return 1

    ser.rts = True
    ser.dtr = True

    print(f"--- {port_name} @ {baud} 8N1, flow control off. Ctrl-] to quit. ---")
    print(f"--- CTS={ser.cts} (False just means the Beast isn't asserting "
          f"RTS; typing still works here) ---")
    print("--- press Enter to wake the prompt ---")

    stdin_fd = sys.stdin.fileno()
    saved = termios.tcgetattr(stdin_fd)
    try:
        tty.setraw(stdin_fd)
        while True:
            ready, _, _ = select.select([stdin_fd, ser.fileno()], [], [], 0.05)

            if stdin_fd in ready:
                data = os.read(stdin_fd, 1024)
                if not data:
                    break
                if QUIT_KEY in data:
                    data = data[:data.index(QUIT_KEY)]
                    if data:
                        ser.write(data)
                        ser.flush()
                    break
                # Enter arrives as \n in raw mode; CP/M wants CR.
                data = data.replace(b'\n', b'\r')
                ser.write(data)
                ser.flush()

            if ser.fileno() in ready:
                chunk = ser.read(4096)
                if chunk:
                    # CP/M sends bare CR and bare LF in both orders; pass
                    # them through untouched so its cursor work looks right.
                    os.write(sys.stdout.fileno(), chunk)
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, saved)
        ser.close()
        print("\r\n--- disconnected, port released ---")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port', help='Serial port (e.g. /dev/ttyUSB2)')
    ap.add_argument('--baud', type=int, default=19200)
    args = ap.parse_args()
    sys.exit(run(args.port, args.baud))


if __name__ == '__main__':
    main()
