# SLIDE — Z80-side requirements for Beastty

**Status:** Pending upstream merge (see §Upstream patch below)
**Date:** 2026-05-08
**Audience:** Z80 firmware authors maintaining `slide.com` on the MicroBeast,
              and Beastty users running the SLIDE protocol against patched
              MicroBeast firmware.

> Beastty is an in-browser VT52 terminal emulator for the MicroBeast Z80
> retrocomputer. The SLIDE protocol layered on top of VT52 enables file
> transfer between the host PC and the Z80. This document specifies the
> Z80-side behaviour Beastty depends on — that is, the deltas a stock
> SLIDE v0.2 implementation must absorb to interoperate with Beastty's
> v0.2.1 sender/receiver.

## 1. Wakeup signature: ESC ^ S L I D E

When the Z80 enters SLIDE mode (either as a result of `B:SLIDE R` /
`B:SLIDE S FILE.TXT`, or as a result of receiving its own ESC^ wakeup
prefix), it MUST emit the 7-byte signature

```
ESC ^ S L I D E   →   0x1B 0x5E 0x53 0x4C 0x49 0x44 0x45
```

verbatim on the serial transmit line before any SLIDE control byte
(`RDY` / `ACK` / `NAK` / `FIN` / `CAN`) or framed payload.

Beastty enters SLIDE recv mode only after detecting this 7-byte prefix
on the inbound serial stream. The byte-loop matcher is torn-chunk safe
— the signature is detected across arbitrary Web Serial chunk
boundaries — so Z80 firmware authors do not need to coalesce the
emission into a single UART write.

Pre-v0.2.1 stock `slide.com` does NOT emit this prefix; modern
patched `slide.com` (post-PR — see §Upstream patch) does. Until the
patch lands, Beastty's `Compatibility mode` Settings option provides
a fallback; see §3 below.

## 2. v0.2.1 amendment: bidirectional CTRL_CAN echo

Per [ADR-003](../.planning/decisions/ADR-003-slide-v0-2-1-can-amendment.md)
(the protocol authority for this amendment): the SLIDE v0.2 spec
defines `CTRL_CAN = 0x18` as a cancellation signal but specifies an
asymmetric contract — only the receiver may emit it. The v0.2.1
amendment closes that gap.

The amended contract:

1. **Either side MAY initiate `CTRL_CAN`.** Raw single byte `0x18` on
   the wire — NOT a wrapped frame. (See ADR-003 §Decision points 1-2
   for the wire-format evidence; the byte is sent identically to
   `RDY` / `FIN` / `ACK` / `NAK`.)
2. **The other side MUST echo `CTRL_CAN` back within ~500 ms.**
3. **Both sides drain the wire and return to idle.** No further
   payload bytes are emitted; both ends are ready for the next
   session.
4. **Idempotent host-initiated cancel.** The PC SM treats a second
   `CTRL_CAN` from the same side as a no-op (per ADR-003 §Decision
   point 3); Z80 firmware should do the same to tolerate user
   double-clicks on Beastty's Cancel chip.

The amendment makes CAN symmetric. Previously only the receiver could
emit it; v0.2.1 lets either side initiate cancellation and demands an
echo from the peer.

If the Z80 cannot echo within 500 ms (older firmware, busy disk I/O),
Beastty applies a 2 s absolute timeout escape hatch (per ADR-003
§Decision point 3 `force_idle`) and returns the wire to a clean state
client-side. Echo is strongly preferred — without it the Beastty user
sees a 2 s wait before the chip clears.

## 3. Send command convention: B:SLIDE R

Beastty's default auto-send command is `B:SLIDE R\r` — drive `B:`,
command `SLIDE`, mode `R` (receive). The Z80 must accept this at the
CP/M prompt and enter SLIDE receive mode within a few hundred
milliseconds — Beastty's wakeup-detection window opens immediately
after the `\r` is placed on the wire, and the Compatibility-mode
default `auto` provides a 3 s timeout fallback for slow Z80 boards
(per Phase 11 D-15).

The auto-send command is configurable in Beastty's `Settings → SLIDE
file transfer` panel. Beastty validates user input against the
client-side safety regex `/^[A-Za-z0-9: ]*\r$/` before any bytes hit
the wire — the command must be alphanumeric, may include `:` or
ASCII space, and must end in a single carriage return. Z80 firmware
authors do not need to sanitize input on their end; unsafe values
never leave the browser.

For the Z80 → PC direction, the established convention is `B:SLIDE S
FILE.TXT` (or `B:SLIDE S FILE.TXT,FILE2.TXT,...` for multi-file
batches). The Z80 emits the wakeup signature from §1, then the
v0.2.1 framed payload; Beastty's recv path consumes both.

## 4. Upstream patch

The Z80-side patch implementing items 1-2 above is tracked at
<https://github.com/blowback/slide>. **Status: pending upstream merge**
as of 2026-05-08 — verified by inspection of `slide.asm` HEAD on the
upstream main branch (no `ESC^SLIDE` emission path; no `CTRL_CAN`
echo path).

Until the patch lands:

- **Send (PC → Z80) works against stock firmware.** Beastty auto-types
  `B:SLIDE R\r`; stock `slide.com` enters receive mode and accepts
  framed payload normally. The wakeup-signature gate is bypassed via
  the Compatibility-mode 3 s timeout fallback (Settings → SLIDE file
  transfer → Compatibility mode = `auto`).
- **Recv (Z80 → PC) is degraded.** Without the wakeup signature
  emission Beastty cannot enter recv mode; the operator must use the
  Compatibility-mode `force-start` option to bypass the gate. Once
  the patch lands, recv "just works" — no operator intervention.
- **Cancellation is asymmetric.** PC-initiated cancel reaches the
  Z80, but the Z80 does not echo back; Beastty's 2 s `force_idle`
  escape hatch returns the wire to idle without confirmation from
  the peer. Z80-initiated cancel is unaffected (this is the only
  direction the v0.2 spec already supported).

Do not hardcode a PR number against this doc — the v0.2.1 amendment
branch may move; readers can find the open work from the upstream
repository root.

## 5. Cross-link

- [ADR-003 — SLIDE v0.2.1 CAN-Bidirectional Amendment](../.planning/decisions/ADR-003-slide-v0-2-1-can-amendment.md)
  — the protocol authority for the amendment summarised in §2.
- [SPEC-v0.2.md](https://github.com/blowback/slide) (in the upstream
  repository) — base v0.2 protocol specification.
- <https://github.com/blowback/slide> — upstream Z80 reference
  implementation; the home of the in-flight v0.2.1 patch branch.
- [.planning/PROJECT.md](../.planning/PROJECT.md) — Beastty project
  context, including the v1.1 FileTransfer milestone scope this
  document is part of.
