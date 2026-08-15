# SLIDE v0.2.2 Specification — Bit-7-Safe Z80 Transmit

## Overview

v0.2.2 is a backward-compatible amendment to [v0.2.1](SPEC-v0.2.1.md). It
adds two things:

1. **Byte-stuffing on Z80 transmit** — the Z80 escapes any byte it sends
   that its BIOS console-output path would otherwise corrupt, so the full
   8-bit wire protocol survives a BIOS that isn't 8-bit clean.
2. **Capability negotiation** — a one-byte flag after the wakeup signature
   tells the PC whether this particular Z80 build escapes its output, so a
   single PC client can auto-detect and interoperate with both this
   (BIOS-console) build and the raw-UART `slide.com` build, which never
   escapes and never sends the flag.

Everything else from v0.2.1 (frame format, session flow, wakeup signature,
bidirectional cancel, sliding-window ACK/NAK, FIN handshake) is unchanged.

## Background

`slidecpm.asm` deliberately routes all serial I/O through the CP/M BIOS
console jump table (CONST/CONIN/CONOUT) instead of raw UART port access, for
portability across different BIOS/host implementations. On at least one
real target (confirmed with the `diagcpm.com` diagnostic against a RunCPM
host), **BIOS CONOUT masks bit 7 of every byte it transmits** — send `0x80`
and `0x00` comes out the wire. BIOS CONIN (receive) is unaffected; a full
8-bit value read from the wire is returned to the caller correctly.

This is a one-directional problem: bytes the Z80 **transmits** may lose bit
7; bytes the Z80 **receives**, and everything the PC transmits (Node's
`serialport` writes go straight to the wire, not through any BIOS), are
unaffected. Left unfixed, this corrupts frame length fields, payload bytes
≥0x80 (any binary file), CRC bytes ≥0x80, and ACK/NAK sequence numbers
≥0x80 — anything the Z80 sends with the top bit set.

## §1. Escaping scheme

Two reserved introducer bytes, both < 0x80 so they always survive the
lossy path themselves:

```text
ESC_HIGH = 0x7E   ; "the next byte is (next_byte | 0x80)"
ESC_LIT  = 0x7D   ; "the next byte is literal, don't reinterpret it"
```

Encoding a byte `b` for transmission:

- `b < 0x80` and `b != ESC_HIGH` and `b != ESC_LIT` → send `b` unchanged
  (1 byte).
- `b == ESC_HIGH` or `b == ESC_LIT` → send `[ESC_LIT, b]` (2 bytes).
- `b >= 0x80` → send `[ESC_HIGH, b & 0x7F]` (2 bytes).

Decoding is a 3-state machine (`normal` / `expect-literal` / `expect-high`)
that persists across transport-chunk boundaries, same as wakeup-signature
detection:

- `normal`, byte `x`: if `x == ESC_LIT` → state `expect-literal`, emit
  nothing. If `x == ESC_HIGH` → state `expect-high`, emit nothing.
  Otherwise emit `x`, stay `normal`.
- `expect-literal`, byte `x`: emit `x` unchanged, state → `normal`.
- `expect-high`, byte `x`: emit `x | 0x80`, state → `normal`.

This is unambiguous for all 256 byte values and needs no lookahead. Worst
case (dense high-bit or 0x7D/0x7E content) is 2x expansion; typical binary
data averages roughly 1.5x.

### Where it applies

- **Z80 side**: every protocol byte send — `send_frame` (SOF/SEQ/LEN/
  payload/CRC), `send_ack`, `send_nak`, `send_rdy`, `send_fin`, `send_can`,
  `respond_to_cancel`, `send_wakeup` — goes through a new `proto_tx`
  wrapper instead of calling `uart_tx` directly. `proto_tx` applies the
  encoding above, then calls `uart_tx` once or twice.
- **NOT applied** to `print_user_msg` (human-readable console text) — it
  keeps calling `uart_tx` directly, unescaped, since a real terminal on the
  other end doesn't know this scheme and would show mangled text if any
  console message happened to contain `0x7D`, `0x7E`, or a byte ≥0x80.
- **PC side**: decoding happens once, at the lowest point raw bytes come
  off the wire (`SerialSession`'s serial-port `data` handler in
  `protocol.ts`), so every consumer — `recvControl`, `recvFrame`, the
  wakeup-signature scan — transparently sees the decoded logical byte
  stream. PC→Z80 traffic is not encoded (unnecessary — that direction is
  already 8-bit clean).

## §2. Capability negotiation

Immediately after the 7-byte wakeup signature (v0.2.1 §1), the Z80 sends
one additional byte:

```text
'7' (0x37)  ⇒  "I escape transmitted protocol bytes per §1"
```

Deliberately a printable ASCII character rather than a small binary value
like `0x01` — a binary value in that range sits uncomfortably close to
`SOF`/`CTRL_FIN`/etc., and reads as opaque noise in a trace dump. `'7'`
shows up as plain text right after `...SLIDE`, and is unambiguous: nothing
else this early in a session is a byte in the printable-ASCII range. Only
this one value is currently defined; anything else received in this
position means "does not escape" (matches the no-byte-sent case below).

### Why this is safe to bolt onto the existing handshake

- `'7'` is well under `0x80` and isn't `ESC_HIGH`/`ESC_LIT` — it never needs
  escaping itself, so it arrives identically whether or not the reader is
  unescaping at that point. No chicken-and-egg problem.
- A PC client that predates this amendment doesn't know to look for it. It
  just sees one more stray byte after the signature and ignores it — the
  same tolerance every v0.2.x receive loop already has for bytes that
  aren't a recognised control code. No parsing changes required for that
  case to stay safe.
- `slide.com` (raw UART, always 8-bit clean) and pre-v0.2.2 `slidecpm.com`
  builds never send this byte at all. A PC client implementing this
  amendment MUST default to "peer does not escape" and only switch to
  unescaping after actually observing `'7'` — defaulting the other way
  would corrupt a literal `0x7D`/`0x7E` data byte from a peer that was
  never escaping in the first place.

### Reference implementation notes

`slide-ts`'s `SerialSession` (`protocol.ts`) watches the raw incoming byte
stream for this sequence continuously, independent of and alongside
whatever the caller (`recv.ts`/`send.ts`) is doing with those same bytes —
so neither handshake function needed any changes. Detecting the capability
byte flips an internal `escapeDecodingEnabled` flag; the unescape state
machine (§1) is a pure passthrough until that flag is set.

## §3. Compatibility

With capability negotiation, one updated PC client interoperates with
both Z80 variants automatically — no user-visible mode switch needed. What
remains genuinely incompatible: a **PC client that predates this amendment**
talking to a **v0.2.2 Z80 build** will still see literal `0x7D`/`0x7E`
escape bytes in frame data and misparse it, since it has no unescape logic
at all. Reference PC clients (`slide-ts`, `slide-py`, `slide-rs`) must be
updated to at least this amendment before talking to `slidecpm.com` v0.5.2.

## §4. Versioning

- Wire protocol version: **v0.2.2**.
- Reference Z80 application: `slidecpm.com` (BIOS-console variant).
- Reference PC application: `slide-ts` (decoding + capability detection
  added to `protocol.ts`). `slide-py` and `slide-rs` have not yet been
  updated for this amendment — Z80 firmware built with `proto_tx` active
  will not interoperate with those two until they are.

## §5. Cross-references

- [SPEC-v0.2.1.md](SPEC-v0.2.1.md) — base spec this amends.
- [SPEC-v0.2.md](SPEC-v0.2.md) — original wire format.
