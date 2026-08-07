#!/usr/bin/env python3
"""
SLIDE - Serial Line Inter-Device (file) Exchange
Command-channel capability probe (wire v0.3 §2).

Sends CTRL_ENQ (0x05) to a peer sitting in receive mode and reports what
comes back. Its main job right now is to confirm the v0.3 §8 compatibility
claim against real v0.2.1 firmware:

  - a v0.2.1 peer skips the byte as stray and answers nothing, and
  - the session is undamaged afterwards, which we demonstrate by completing
    a normal FIN exchange (and optionally a whole file transfer) once the
    probe has come back empty.

Usage: slide-probe <serial_port> [--baud 19200] [--then-send FILE]
"""

import argparse
import os
import sys

from slide.common import (
    CTRL_ENQ, CTRL_FIN, CTRL_CAN,
    CMDSET_MAJOR, CMDSET_MINOR,
    ProbeOutcome,
    open_serial, handshake_as_sender, probe_command_support, recv_control,
)


def _hex(b: bytes) -> str:
    return b.hex(' ') if b else '(none)'


def report(result) -> None:
    """Print the probe verdict and the raw evidence behind it."""
    print()
    print("--- Probe result ---")
    print(f"  Outcome:  {result.outcome.value}")
    print(f"  Attempts: {result.attempts}")
    print(f"  Stray bytes during probe window: {_hex(result.stray)}")

    if result.outcome is ProbeOutcome.SUPPORTED:
        print(f"  VER raw:  {_hex(result.ver_raw)} ('{result.ver_raw.decode('ascii')}')")
        print(f"  Version:  major {result.major}, minor {result.minor}")
        print(f"  We speak: major {CMDSET_MAJOR}, minor {CMDSET_MINOR}")
        if not result.usable:
            print()
            print("  Peer's major version is not one we know. Per v0.3 §2, opcode")
            print("  and record encodings may have changed, so commands MUST NOT")
            print("  be sent. Falling back as if there had been no echo.")
        elif result.minor > CMDSET_MINOR:
            print()
            print("  Peer is a newer minor. The command set is additive within a")
            print("  major, so every opcode we know still means what we think.")
        elif result.minor < CMDSET_MINOR:
            print()
            print("  Peer is an older minor. Restrict to opcodes defined at or")
            print(f"  below minor {result.minor}.")

    elif result.outcome is ProbeOutcome.UNSUPPORTED:
        print()
        print("  No echo. This is the expected v0.2.1 result: the peer skipped")
        print("  0x05 as a stray byte and stayed in its file loop. Commands MUST")
        print("  NOT be sent (v0.3 §6) — an unannounced frame would be parsed as")
        print("  a file header and would put junk on the disk.")

    elif result.outcome is ProbeOutcome.MALFORMED:
        print(f"  VER raw:  {_hex(result.ver_raw)}")
        print()
        print("  ENQ was echoed but VER did not arrive as four ASCII digits.")
        print("  v0.3 §2 says treat this as no command support rather than")
        print("  guessing at the version.")

    elif result.outcome is ProbeOutcome.CANCELLED:
        print()
        print("  Peer cancelled during the probe. CAN has been echoed and the")
        print("  wire drained (v0.2.1 §2); both sides are back at idle.")

    if result.detail:
        print(f"  Detail:   {result.detail}")
    print()


def probe_session(port: str, baud: int = 19200, attempts: int = 3,
                  echo_timeout: float = 0.5, settle: float = 0.1,
                  then_send: str = None, skip_fin: bool = False,
                  debug: bool = False) -> int:
    """Handshake, probe, optionally transfer a file, then close the session."""

    print("SLIDE - command-channel probe (wire v0.3 §2)")
    print(f"  Port: {port} @ {baud} baud")
    print(f"  Probe: ENQ 0x{CTRL_ENQ:02X}, {attempts} attempt(s), "
          f"{echo_timeout:.1f}s echo timeout")
    print()

    ser = open_serial(port, baud)

    print("Waiting for Z80 (start SLIDE R on Z80 now)...")
    if not handshake_as_sender(ser, timeout=60.0):
        print("ERROR: no RDY echo from Z80 (handshake timeout or cancel).")
        ser.close()
        return 1
    print("Z80 connected.")

    # The probe goes exactly where a file header or FIN would go.
    result = probe_command_support(ser, attempts=attempts,
                                   echo_timeout=echo_timeout,
                                   settle=settle, debug=debug)
    report(result)

    if result.outcome is ProbeOutcome.CANCELLED:
        ser.close()
        return 1

    # --- Prove the session survived the probe ---
    session_ok = True

    if then_send:
        print(f"--- Sending {then_send} to confirm the session still works ---")
        from slide.send import send_file
        if not send_file(ser, then_send, debug):
            print("  File transfer FAILED after the probe.")
            session_ok = False
        print()

    if skip_fin:
        print("Skipping FIN exchange (--no-fin). The Z80 is still in its file loop.")
        ser.close()
        return 0 if session_ok else 1

    print("--- FIN exchange (session-intact check) ---")
    ser.write(bytes([CTRL_FIN]))
    ser.flush()
    try:
        ctrl, _ = recv_control(ser, timeout=5.0)
    except TimeoutError:
        ctrl = None

    if ctrl == CTRL_FIN:
        print("  FIN echoed. The peer was still in its file loop and exited")
        print("  cleanly — the probe did not disturb the session.")
    elif ctrl == CTRL_CAN:
        print("  Peer cancelled instead of echoing FIN.")
        session_ok = False
    elif ctrl is None:
        print("  No FIN echo within 5s. The probe may have disturbed the peer —")
        print("  worth investigating before trusting the compatibility claim.")
        session_ok = False
    else:
        print(f"  Unexpected reply to FIN: 0x{ctrl:02X}")
        session_ok = False

    print()
    if session_ok:
        print(f"VERDICT: probe outcome '{result.outcome.value}', session intact.")
    else:
        print(f"VERDICT: probe outcome '{result.outcome.value}', "
              f"but the session did NOT close cleanly.")

    ser.close()
    return 0 if session_ok else 1


def main():
    parser = argparse.ArgumentParser(
        description='SLIDE - probe a peer for v0.3 command-channel support')
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--baud', type=int, default=19200,
                        help='Baud rate (default: 19200)')
    parser.add_argument('--attempts', type=int, default=3,
                        help='Probe attempts before giving up (default: 3)')
    parser.add_argument('--timeout', type=float, default=0.5,
                        help='Seconds to wait for each echo (default: 0.5)')
    parser.add_argument('--settle', type=float, default=0.1,
                        help='Seconds to wait after handshake before probing, '
                             'to clear the post-RDY FIFO flush (default: 0.1)')
    parser.add_argument('--then-send', metavar='FILE',
                        help='After probing, send FILE to prove the session still works')
    parser.add_argument('--no-fin', action='store_true',
                        help='Leave the peer in its file loop instead of closing')
    parser.add_argument('--debug', action='store_true',
                        help='Show wire-level debug output')
    args = parser.parse_args()

    if args.then_send and not os.path.exists(args.then_send):
        print(f"Error: file '{args.then_send}' not found")
        sys.exit(1)

    sys.exit(probe_session(args.port, args.baud, args.attempts, args.timeout,
                           args.settle, args.then_send, args.no_fin, args.debug))


if __name__ == '__main__':
    main()
