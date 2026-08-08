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
    open_serial, handshake_as_sender, send_start_command,
    probe_command_support, recv_control,
    CMD_NOP, CMD_VOLS, CMD_DIR, send_command, status_name,
)


def _hex(b: bytes) -> str:
    return b.hex(' ') if b else '(none)'


def _drive_list(bitmap: int) -> str:
    """Render a drive bitmap as CP/M drive letters."""
    drives = [f"{chr(ord('A') + n)}:" for n in range(16) if bitmap & (1 << n)]
    return ' '.join(drives) if drives else '(none)'


def show_vols(records: bytes) -> None:
    """Pretty-print a CMD_VOLS reply: one 5-byte record (v0.3 §5)."""
    if len(records) < 5:
        print(f"  Malformed: expected a 5-byte record, got {len(records)}")
        print(f"  Raw: {_hex(records)}")
        return
    present = records[0] | (records[1] << 8)
    logged = records[2] | (records[3] << 8)
    print(f"  Present: {_drive_list(present)}")
    print(f"  Logged:  {_drive_list(logged)}")
    print(f"  Current: {chr(ord('A') + min(records[4], 15))}:")


def show_dir(records: bytes) -> None:
    """Pretty-print a CMD_DIR reply: 16-byte records (v0.3 §5)."""
    if len(records) % 16:
        print(f"  Malformed: {len(records)} bytes is not a whole number of "
              f"16-byte records")
        print(f"  Raw: {_hex(records)}")
        return
    if not records:
        print("  (empty directory)")
        return
    print(f"  {'USER':<4} {'NAME':<12} {'ATTR':<5} {'BYTES':>10}")
    for i in range(0, len(records), 16):
        r = records[i:i + 16]
        name = r[1:9].decode('ascii', 'replace').rstrip()
        ext = r[9:12].decode('ascii', 'replace').rstrip()
        attr = ''.join(f for bit, f in ((0x01, 'R/O '), (0x02, 'SYS '),
                                        (0x04, 'ARC ')) if r[12] & bit)
        # Size is in 128-byte records — an upper bound in bytes, as for the
        # file-transfer header's size field.
        recs = r[13] | (r[14] << 8) | (r[15] << 16)
        print(f"  {r[0]:<4} {name + '.' + ext:<12} {attr.rstrip():<5} "
              f"{recs * 128:>10}")


def report(result, label: str) -> None:
    """Print the probe verdict and the raw evidence behind it."""
    print()
    print(f"--- Probe result ({label}) ---")
    print(f"  Outcome:  {result.outcome.value}")
    print(f"  Attempts: {result.attempts}")
    print(f"  Queued before probe: {_hex(result.discarded)}")
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


def parse_drive(arg) -> int:
    """
    Parse a CP/M drive from the command line: "a", "A:", or a bare 0-15.
    Returns 0..15, or 0xFF for "whatever the Beast currently has selected"
    — the value v0.3 §5 reserves for that.
    """
    if arg is None:
        return 0xFF
    t = arg.strip().rstrip(':')
    if not t:
        raise ValueError(f"empty drive: {arg!r}")
    if len(t) == 1 and t.isalpha():
        n = ord(t.upper()) - ord('A')
        if n > 15:
            raise ValueError(f"drive out of range: {arg!r} (A-P)")
        return n
    if not t.isdigit():
        raise ValueError(f"cannot read {arg!r} as a drive — use A-P or 0-15")
    n = int(t)
    if n > 15:
        raise ValueError(f"drive out of range: {arg!r} (0-15)")
    return n


def drive_label(d: int) -> str:
    """Human-readable name for a drive operand."""
    return 'current drive' if d == 0xFF else f"{chr(ord('A') + d)}:"


def _honour_enq_obligation(ser, result, cmd: str, drive: int, user: int,
                           debug: bool) -> bool:
    """
    v0.3 §2: an echo obliges the client to send exactly one command frame.
    Even when we only wanted to probe, the server is in command mode and
    would read a file header as a command — so complete the exchange with
    CMD_NOP. Returns False if the command itself failed.
    """
    if not result.usable:
        return True

    # §5 operands: [drive][user], 0xFF for "whatever is current"
    opcode, operands, label = {
        'vols': (CMD_VOLS, b'', 'CMD_VOLS'),
        'dir': (CMD_DIR, bytes([drive, user]),
                f'CMD_DIR ({drive_label(drive)})'),
    }.get(cmd, (CMD_NOP, b'', 'CMD_NOP'))

    print(f"--- {label} ---")
    try:
        reply = send_command(ser, opcode, operands, debug)
        print(f"  Status: 0x{reply.status:02X} — {status_name(reply.status)}")
        if reply.status == 0:
            if opcode == CMD_VOLS:
                show_vols(reply.records)
            elif opcode == CMD_DIR:
                show_dir(reply.records)
            else:
                print("  (no records — probe-only exchange completed)")
        elif reply.records:
            print(f"  Unexpected records with a non-OK status: {_hex(reply.records)}")
        print()
        return True
    except Exception as e:
        print(f"  Command FAILED: {e}")
        print()
        return False


def probe_session(port: str, baud: int = 19200, attempts: int = 3,
                  echo_timeout: float = 0.5, settle: float = 0.1,
                  start_cmd: str = None, cmd: str = 'nop',
                  drive: str = None, user: str = None,
                  then_send: str = None,
                  probe_after: bool = False, skip_fin: bool = False,
                  debug: bool = False) -> int:
    """Handshake, probe, optionally transfer a file, then close the session."""

    print("SLIDE - command-channel probe (wire v0.3 §2)")
    print(f"  Port: {port} @ {baud} baud")
    print(f"  Probe: ENQ 0x{CTRL_ENQ:02X}, {attempts} attempt(s), "
          f"{echo_timeout:.1f}s echo timeout")
    print()

    ser = open_serial(port, baud)

    if start_cmd:
        print(f"Typing {start_cmd!r} at the CP/M prompt...")
        send_start_command(ser, start_cmd, debug)
    else:
        print("Waiting for Z80 (start SLIDE R on Z80 now)...")

    hs = handshake_as_sender(ser, timeout=60.0, debug=debug)
    if not hs.connected:
        print("ERROR: no RDY echo from Z80 (handshake timeout or cancel).")
        ser.close()
        return 1
    print(f"Z80 connected. ({hs.strays} stray byte(s) skipped, "
          f"wakeup signature {'seen' if hs.wakeup_seen else 'not seen'})")

    # The probe goes exactly where a file header or FIN would go.
    result = probe_command_support(ser, attempts=attempts,
                                   echo_timeout=echo_timeout,
                                   settle=settle, debug=debug)
    report(result, "after handshake")

    if result.outcome is ProbeOutcome.CANCELLED:
        ser.close()
        return 1

    # v0.3 §2: an echo obliges us to send exactly one command frame.
    session_ok = _honour_enq_obligation(ser, result, cmd,
                                        parse_drive(drive), parse_drive(user),
                                        debug)

    if then_send:
        print(f"--- Sending {then_send} to confirm the session still works ---")
        from slide.send import send_file
        if not send_file(ser, then_send, debug):
            print("  File transfer FAILED after the probe.")
            session_ok = False
        print()

    # v0.3 §2 lists three legal probe positions. The one above covers
    # "right after the RDY handshake"; this covers "after a completed file
    # transfer", where the residue risk is different — in receive mode the
    # preceding transfer's ACKs flow *from* the server, so anything the
    # client left unread lands in this probe's window.
    if probe_after:
        if not then_send:
            print("Note: --probe-after without --then-send re-probes the same position.")
        after = probe_command_support(ser, attempts=attempts,
                                      echo_timeout=echo_timeout,
                                      settle=settle, debug=debug)
        report(after, "after file transfer")
        if after.outcome is ProbeOutcome.CANCELLED:
            ser.close()
            return 1
        if after.outcome is not result.outcome:
            print("  NOTE: post-transfer outcome differs from the post-handshake one.")
        if not _honour_enq_obligation(ser, after, 'nop', 0xFF, 0xFF, debug):
            session_ok = False
        if after.discarded:
            print(f"  NOTE: {len(after.discarded)} byte(s) were still queued "
                  f"from the transfer.")
        print()

    if skip_fin:
        print("Skipping FIN exchange (--no-fin). The Z80 is still in its file loop.")
        ser.close()
        return 0 if session_ok else 1

    print("--- FIN exchange (session-intact check) ---")
    ser.write(bytes([CTRL_FIN]))
    ser.flush()
    # 10s, not 5: the receiver closes the file (a directory write) after
    # acknowledging EOF and before returning to its file loop.
    try:
        ctrl, _ = recv_control(ser, timeout=10.0)
    except TimeoutError:
        ctrl = None

    if ctrl == CTRL_FIN:
        print("  FIN echoed. The peer was still in its file loop and exited")
        print("  cleanly — the probe did not disturb the session.")
    elif ctrl == CTRL_CAN:
        print("  Peer cancelled instead of echoing FIN.")
        session_ok = False
    elif ctrl is None:
        print("  No FIN echo within 10s. The probe may have disturbed the peer —")
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
    parser.add_argument('--timeout', type=int, default=500,
                        help='Milliseconds to wait for each echo (default: 500)')
    parser.add_argument('--settle', type=int, default=100,
                        help='Milliseconds to wait after handshake before '
                             'probing, to clear the post-RDY FIFO flush '
                             '(default: 100)')
    parser.add_argument('--start-cmd', metavar='CMD',
                        help="Type this at the peer's CP/M prompt to start it, "
                             "instead of requiring a separate terminal "
                             "(e.g. 'slide r' or 'b:slide r')")
    parser.add_argument('--cmd', default='nop', choices=['nop', 'vols', 'dir'],
                        help='Command to issue if the peer supports them '
                             '(default: nop)')
    parser.add_argument('--drive',
                        help='Drive to list with --cmd dir: A-P or 0-15. Omit '
                             'for the drive the MicroBeast currently has selected')
    parser.add_argument('--user',
                        help='User number to list with --cmd dir: 0-15. Omit '
                             'for the current one')
    parser.add_argument('--then-send', metavar='FILE',
                        help='After probing, send FILE to prove the session still works')
    parser.add_argument('--probe-after', action='store_true',
                        help='Probe again after the file transfer, covering the '
                             'v0.3 §2 post-transfer position (use with --then-send)')
    parser.add_argument('--no-fin', action='store_true',
                        help='Leave the peer in its file loop instead of closing')
    parser.add_argument('--debug', action='store_true',
                        help='Show wire-level debug output')
    args = parser.parse_args()

    if args.then_send and not os.path.exists(args.then_send):
        print(f"Error: file '{args.then_send}' not found")
        sys.exit(1)

    sys.exit(probe_session(args.port, args.baud, args.attempts,
                           args.timeout / 1000.0, args.settle / 1000.0,
                           args.start_cmd, args.cmd,
                           args.drive, args.user, args.then_send,
                           args.probe_after, args.no_fin, args.debug))


if __name__ == '__main__':
    main()
