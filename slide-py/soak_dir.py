#!/usr/bin/env python3
"""
soak_dir.py — hunt the intermittent CMD_DIR failure.

Runs the full probe → CMD_DIR → FIN exchange in a loop until it fails,
then runs the decisive follow-up: send a SECOND FIN.

  - If the second FIN is echoed, the Z80 was alive in .file_loop all along
    and the FIRST FIN was swallowed. That points at the uart_flush_rx in
    serve_command's .done path (v0.3 §9), which drains whatever is in the
    FIFO right after reading the terminator ACK.
  - If nothing comes back, the Z80 stopped servicing the UART — a crash or
    a blocking BIOS console write, not a lost byte.

Usage: soak_dir.py /dev/ttyUSB1 [--runs 40] [--cmd-name "slide r"]
"""

import argparse
import sys
import time

from slide.common import (
    CTRL_FIN, CTRL_CAN, CMD_DIR, CMD_NOP,
    open_serial, send_start_command, handshake_as_sender,
    probe_command_support, send_command, recv_control, drain_and_capture,
)


def one_run(port: str, baud: int, start_cmd: str, run_no: int) -> tuple:
    """Returns (ok, detail). Leaves the port closed."""
    ser = open_serial(port, baud)
    try:
        send_start_command(ser, start_cmd)
        hs = handshake_as_sender(ser, timeout=60.0)
        if not hs.connected:
            return (False, "no RDY echo (handshake)")

        probe = probe_command_support(ser)
        if not probe.usable:
            return (False, f"probe outcome {probe.outcome.value}")

        reply = send_command(ser, CMD_DIR, bytes([0xFF, 0xFF]))
        if reply.status != 0:
            return (False, f"CMD_DIR status 0x{reply.status:02X}")
        nrec = len(reply.records) // 16

        # --- the FIN exchange, which is where it fails ---
        t0 = time.time()
        ser.write(bytes([CTRL_FIN]))
        ser.flush()
        try:
            ctrl, _ = recv_control(ser, timeout=10.0)
        except TimeoutError:
            ctrl = None

        if ctrl == CTRL_FIN:
            return (True, f"{nrec} records, FIN echoed in {time.time()-t0:.2f}s")

        # --- FAILED. Now the decisive test. ---
        print(f"\n=== run {run_no} FAILED: first FIN drew "
              f"{'nothing' if ctrl is None else hex(ctrl)} ===", flush=True)

        leftovers = drain_and_capture(ser, quiet=0.2, cap=2.0)
        print(f"    bytes queued after the failure: {leftovers.hex(' ') or '(none)'}",
              flush=True)

        print("    sending a SECOND FIN ...", flush=True)
        ser.write(bytes([CTRL_FIN]))
        ser.flush()
        try:
            ctrl2, _ = recv_control(ser, timeout=10.0)
        except TimeoutError:
            ctrl2 = None

        if ctrl2 == CTRL_FIN:
            verdict = ("SECOND FIN ECHOED -> the Z80 was alive in .file_loop; "
                       "the FIRST FIN was swallowed (uart_flush_rx in .done)")
        elif ctrl2 is None:
            verdict = ("second FIN also ignored -> the Z80 stopped servicing "
                       "the UART (crash or blocking console write)")
        else:
            verdict = f"second FIN drew 0x{ctrl2:02X}"
        print(f"    VERDICT: {verdict}", flush=True)
        return (False, verdict)
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port')
    ap.add_argument('--baud', type=int, default=19200)
    ap.add_argument('--runs', type=int, default=40)
    ap.add_argument('--cmd-name', default='slide r',
                    help="what to type at the CP/M prompt (default: 'slide r')")
    args = ap.parse_args()

    for i in range(1, args.runs + 1):
        ok, detail = one_run(args.port, args.baud, args.cmd_name, i)
        print(f"{i:3d} {'ok  ' if ok else 'FAIL'}  {detail}", flush=True)
        if not ok:
            print(f"\nStopped after {i} run(s). The Beast is probably wedged now — "
                  f"power-cycle it, then use wake_beast.py before the next attempt.")
            return 1
        time.sleep(0.3)

    print(f"\n{args.runs} runs, no failures.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
