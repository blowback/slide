# SLIDE v0.2.1 Specification — Wakeup Signature & Bidirectional Cancel

## Overview

v0.2.1 is a backward-compatible amendment to [v0.2](SPEC-v0.2.md). It adds:

1. **Wakeup signature** — Z80 emits a 7-byte `ESC ^ S L I D E` prefix on
   entering SLIDE mode, so the host can detect that `slide.com` is alive
   without polling.
2. **Bidirectional `CTRL_CAN`** — either side may initiate cancellation;
   the other side MUST echo `CTRL_CAN` back within ~500 ms. Idempotent.

Everything else from v0.2 (frame format, session flow, multi-file support,
CRC, sliding-window ACK/NAK, FIN handshake) is unchanged.

A v0.2 implementation will interoperate with a v0.2.1 implementation in
the common case (no cancel) — the wakeup signature bytes are not control
bytes and are skipped by v0.2's "ignore stray bytes" wait loops. v0.2
behaviour during cancel is degraded (no echo, no clean wire drain) but
not catastrophic.

## §1. Wakeup signature: `ESC ^ S L I D E`

When the Z80 enters SLIDE mode (either via `SLIDE`, `SLIDE R`, or
`SLIDE S FILE.TXT` from the CP/M prompt), it MUST emit the 7-byte
signature

```
ESC ^ S L I D E   →   0x1B 0x5E 0x53 0x4C 0x49 0x44 0x45
```

verbatim on the serial transmit line **before any SLIDE control byte**
(`RDY` / `ACK` / `NAK` / `FIN` / `CAN`) **or framed payload**.

### Rationale

The host (e.g. an in-browser terminal emulator) needs a deterministic
signal that `slide.com` has booted and is in SLIDE mode. Polling RDY
works for the send path (host is the active sender) but not for the
receive path (host must be in receive mode before the Z80 starts
sending). The wakeup signature lets the host enter receive mode the
moment it sees the prefix, with no race window.

### Detection

The signature MAY be detected across arbitrary transport-chunk
boundaries — implementations should match it as a byte sequence, not
require it to arrive in a single read. The choice of `ESC ^` (`0x1B
0x5E`) as the prefix is deliberate: it's a benign VT52 sequence
(unrecognised escape) so it has no visible effect on terminals that
accidentally render the bytes.

### Compatibility

Pre-v0.2.1 `slide.com` does NOT emit this prefix. Hosts that depend on
the signature SHOULD provide a fallback mode (timeout-based or operator
override) for talking to v0.2 firmware.

## §2. Bidirectional `CTRL_CAN` (cancel)

v0.2 defined `CTRL_CAN = 0x18` as an asymmetric cancel signal — only the
receiver could emit it, used by the Z80 to signal disk-write failure.
v0.2.1 makes the contract symmetric.

### The amended contract

1. **Either side MAY initiate `CTRL_CAN`.** Raw single byte `0x18` on
   the wire — NOT a wrapped frame. Sent identically to `RDY` / `FIN`
   / `ACK` / `NAK`.
2. **The other side MUST echo `CTRL_CAN` back within ~500 ms.**
3. **Both sides drain the wire and return to idle.** No further payload
   bytes are emitted; both ends are ready for the next session.
4. **Idempotent.** A second `CTRL_CAN` from the same side is a no-op.
   This tolerates a user double-clicking a Cancel button on the host or
   a duplicated send on the Z80.

### Implementation pattern

Each side keeps a per-session `cancel_initiated` flag:

- **Initiating cancel:** set `cancel_initiated`, send `CTRL_CAN`, wait
  briefly for the peer's echo (any incoming `CTRL_CAN` while this flag
  is set is the peer's echo and MUST NOT be re-echoed), then drain the
  input buffer and return to idle.
- **Responding to peer-initiated cancel:** if `cancel_initiated` is
  clear, echo `CTRL_CAN` back, set `cancel_initiated` (so any further
  `CTRL_CAN` is a no-op), drain, return to idle.

### Where `CTRL_CAN` may be detected

A v0.2.1 implementation MUST look for `CTRL_CAN` everywhere it reads a
raw byte from the wire, not only at frame boundaries. In particular:

- During the RDY handshake loop (either direction).
- While waiting for `SOF` / `FIN` between frames.
- In `recv_control` (the ACK/NAK wait after a sent window).
- During the FIN exchange.

`CTRL_CAN` SHOULD NOT be expected mid-payload; senders MUST NOT
interleave a raw `CTRL_CAN` between the bytes of a frame. This keeps
detection cheap (one byte test in the wait loops) and avoids
ambiguity with payload byte `0x18`.

### Timeout escape hatch

If the peer fails to echo within the implementation's grace window, the
initiating side SHOULD return the wire to a clean idle state on its
own (drain input, drop the session). Recommended grace window: ~500 ms
for the echo, with a hard ceiling of ~2 s before unilateral idle. The
initiating side MUST NOT block forever waiting for an echo.

### Rationale

v0.2's asymmetric cancel meant the host had no way to abort a stuck
transfer cleanly — its only options were to disconnect the serial port
or to send bytes the Z80 would interpret as garbage frames. v0.2.1
gives the host a clean, in-band cancel and lets either side close out
without leaving the other waiting.

## §3. Wire format (unchanged)

```
[SOF=0x01] [SEQ] [LEN_H] [LEN_L] [PAYLOAD...] [CRC_H] [CRC_L]
```

Sequence numbers reset to 0 for each file's header frame, then 1, 2, 3…
for data frames. CRC-16-CCITT (polynomial `0x1021`, init `0xFFFF`) over
`SEQ + LEN + PAYLOAD`.

### Control bytes

| Byte | Name       | Meaning                                          |
|------|------------|--------------------------------------------------|
| 0x01 | SOF        | Start of frame                                   |
| 0x04 | CTRL_FIN   | Session complete, no more files                  |
| 0x06 | CTRL_ACK   | Acknowledge (followed by 1-byte SEQ)             |
| 0x11 | CTRL_RDY   | Ready / handshake                                |
| 0x15 | CTRL_NAK   | Negative acknowledge (followed by 1-byte SEQ)    |
| 0x18 | CTRL_CAN   | Cancel — bidirectional in v0.2.1, see §2         |

## §4. Session flow (unchanged from v0.2)

### Multi-file send

```
SENDER                          RECEIVER
  |                                |
  |  --- (wakeup if Z80) -------> |   (Z80-only, see §1)
  |  --- RDY --->                  |   (sender ready)
  |  <--- RDY ---                  |   (receiver ready)
  |                                |
  |  --- header frame (file 1) --> |
  |  <--- ACK ---                  |
  |  --- data frames ----------->  |
  |  --- EOF frame (len=0) ------> |
  |  <--- ACK ---                  |
  |                                |
  |  --- header frame (file 2) --> |
  |   ... (per file) ...           |
  |                                |
  |  --- FIN --->                  |   (no more files)
  |  <--- FIN ---                  |   (acknowledged)
```

### Cancel (new in v0.2.1, can interrupt any state above)

```
INITIATOR                        PEER
  |                                |
  |  --- CAN --->                  |   (initiator wants out)
  |  <--- CAN ---                  |   (peer echoes within ~500 ms)
  |                                |
  |        (both drain wire and return to idle)
```

### Startup handshake — direction negotiation (unchanged)

Both sides exchange RDY. The sender transmits first, the receiver
responds:

- **PC sending to Z80** (`slide-send` + `SLIDE R`): PC sends RDY → Z80
  echoes RDY → PC begins sending header for first file.
- **Z80 sending to PC** (`SLIDE S file1.com file2.com` + `slide-recv`):
  Z80 emits wakeup → Z80 sends RDY → PC echoes RDY → Z80 begins
  sending header for first file.

## §5. Implementation notes

### Z80 (`slide.com`)

- Emit wakeup at the top of `recv_session` and `send_session`, before
  any RDY.
- Add a `cancel_initiated` byte in static state; reset at the top of
  each session.
- After sending CAN, wait briefly for the echo with the existing
  `uart_rx_timeout` (~2 s upper bound), then `uart_flush_rx`.
- After receiving CAN, echo back via `uart_tx`, set `cancel_initiated`,
  `uart_flush_rx`.
- In multi-file send mode, after each `send_file_tx` returns, check
  `cancel_initiated` and bail on the remaining files — don't send
  headers into a peer that has gone idle.

### PC (`slide-send` / `slide-recv`)

- The wakeup signature bytes are non-control and are naturally
  skipped by existing "ignore stray bytes" loops in `recv_control` and
  the SOF wait. Implementations MAY actively detect the signature as
  a liveness signal but are not required to.
- `recv_control` SHOULD auto-echo `CTRL_CAN` and drain before
  returning a "cancelled" result, so callers can treat it as a
  confirmed cancel.
- The SOF-wait loop SHOULD detect `CTRL_CAN`, echo, drain, and signal
  cancel up to the file-receive loop.
- The PC initiates cancel rarely in batch mode (no UI), but MUST
  respond to peer-initiated cancel correctly. Interactive PC clients
  (e.g. a terminal emulator with a Cancel button) SHOULD also support
  initiating.

## §6. Versioning

- Wire protocol version: **v0.2.1**.
- Reference Z80 application: `slide.com` v0.5 (first version
  implementing wire v0.2.1).
- Reference PC applications: `slide-py`, `slide-rs` — both updated
  alongside v0.5 of the Z80 firmware.

A v0.2 peer can complete a normal (no-cancel) transfer with a v0.2.1
peer. Mixing during cancel is degraded:

- v0.2.1 initiator + v0.2 peer: initiator times out waiting for echo
  and falls through to its unilateral idle drain.
- v0.2 initiator + v0.2.1 peer: peer echoes (harmless to v0.2 since
  v0.2 will be aborting and not reading further).
- v0.2 ↔ v0.2 cancel: as before — receiver-initiated only, no echo,
  initiator drops the session.

## §7. Cross-references

- [SPEC-v0.2.md](SPEC-v0.2.md) — base v0.2 specification this amends.
- [SLIDE_Z80_REQUIREMENT.md](SLIDE_Z80_REQUIREMENT.md) — host-side
  (Beastty) consumer requirements that drove this amendment.
