# SLIDE v0.3 Specification — Remote Filing Commands

## Overview

v0.3 is a backward-compatible amendment to [v0.2.1](SPEC-v0.2.1.md). It
adds a **command channel**: a way for the active side of a session to ask
the passive side a question and receive a framed answer, without
transferring a file.

Two commands are defined:

1. **`CMD_VOLS`** — which volumes (CP/M drives) does the remote have?
2. **`CMD_DIR`** — list the files on a volume, with sizes.

Everything else from v0.2.1 (frame format, CRC, sliding-window ACK/NAK,
session flow, wakeup signature, bidirectional cancel) is unchanged.

### Compatibility floor

This spec targets **wire protocol v0.2.1** — reference firmware
`slide.com` v0.5.0 and later. Pre-v0.2.1 peers (the `slide.com` 0.0.x
series, wire v0.2) are out of scope and need not be supported.

Note that the repository's version numbers do not share a namespace:
`slide.asm` is at v0.5.2, `slide-py` at 0.2.0, `slide-rs` at 0.0.4, and
the wire protocol at v0.2.1. Implementers should key compatibility
decisions to the **wire** version, which is the only number all three
codebases agree on.

## §1. Roles

The command channel is asymmetric. Exactly one side issues commands:

- **Client** — the side that initiates. In practice the PC-side host
  (Beastty, `slide-py`, `slide-rs`).
- **Server** — the side that answers. In practice the Z80 running
  `SLIDE R`, sitting in its inter-frame wait loop.

Commands are only valid while the server is in **receive mode**. A Z80
running `SLIDE S` is driving the session and its only wait point is
`recv_control_z80`, which accepts ACK/NAK/FIN/CAN and nothing else;
sending it a command achieves nothing. Clients MUST NOT attempt commands
against a peer in send mode.

Z80-as-client is symmetric in principle but is not specified here. It is
reserved for a future amendment.

## §2. `CTRL_ENQ` — command prefix and capability probe

### The byte

| Byte | Name     | Meaning                          |
|------|----------|----------------------------------|
| 0x05 | CTRL_ENQ | A command frame follows          |

`0x05` is unused by v0.2.1 and is skipped as a stray byte by every
v0.2.1 wait loop, in both implementations. Sending it to a v0.2.1 peer
is inert.

### The exchange

`CTRL_ENQ` precedes **every** command frame, not just the first. It
serves two purposes at once:

1. It tells the server that the next `SOF` frame is a command frame and
   not a file header — the server has no other way to tell them apart.
2. Its echo (or the absence of one) is the capability probe.

```
CLIENT                              SERVER
  |  --- ENQ --->                     |
  |  <--- ENQ, VER (4 bytes) ---      |   v0.3+ answers within ~500 ms
  |                                   |   v0.2.1 stays silent
```

### `VER` — command-set version

`VER` is four bytes, transmitted as ASCII decimal digits in the form
`MMmm`:

| Field | Bytes | Range       | Meaning              |
|-------|-------|-------------|----------------------|
| `MM`  | 2     | `'00'`–`'99'` | Major — incompatible changes |
| `mm`  | 2     | `'00'`–`'99'` | Minor — additive changes     |

This spec is version **`'0001'`** — major 0, minor 1.

All four bytes lie in `0x30`–`0x39`, disjoint from every control byte, so
a late or duplicated echo can never be mistaken for a control byte by a
client that has already moved on.

### Reading and acting on `VER`

- A client MUST read exactly four bytes following the echo, with a
  ~500 ms timeout, and MUST verify all four are ASCII digits. A short
  read or a non-digit is a malformed echo: treat the peer as having no
  command support and fall back per §2 "No echo". Do not guess.
- **An unknown major is fatal to the command channel.** Opcode
  numbering, operand layout and record formats may all have changed, so a
  client that does not know the major version MUST NOT send command
  frames. Fall back exactly as for no echo.
- **A higher minor is safe.** Within a major version the command set is
  strictly additive: an opcode, operand layout, record format or status
  code defined at minor *m* keeps its meaning at every minor greater than
  *m*. A client MAY use every opcode it knows, and MUST ignore opcodes,
  status codes and trailing record fields it does not.
- **A lower minor** means the server predates some of what the client
  knows. The client SHOULD restrict itself to opcodes defined at or below
  the server's minor. Getting this wrong is recoverable — an
  unimplemented opcode draws `ST_OPCODE` (§4) — but wasteful.

Because `CTRL_ENQ` precedes every command frame, `VER` is repeated on
every command. At five bytes per exchange that is not worth optimising
away, and a constant, unconditional reply keeps the server stateless.

### Where the client may send it

Only where the server is parked in a raw-byte wait loop with no frame in
progress. In v0.2.1 receive mode that means the file loop
(`slide.asm:881`) — that is, in the position where a file header frame
or `CTRL_FIN` would otherwise go:

- immediately after the RDY handshake completes, or
- after a completed file transfer's EOF frame has been ACKed, or
- after a completed command exchange.

A client MUST NOT send `CTRL_ENQ` mid-frame or between a control byte
and its sequence byte.

### Timing

The server flushes its receive FIFO immediately after echoing RDY
(`uart_flush_rx` at `slide.asm:876`), and `slide-py`'s receiver does the
equivalent (`sleep(0.05)` then `reset_input_buffer()`). A probe fired the
instant the RDY echo arrives can be discarded by that flush.

Clients MUST therefore either wait ~100 ms after the handshake before the
first `CTRL_ENQ`, or retry the probe. Recommended: up to **3 attempts,
500 ms apart**.

### Idempotency and draining

A server echoes every `CTRL_ENQ` it receives. A client that retried may
therefore see more than one echo. After the first complete echo (the
`CTRL_ENQ` plus its four `VER` bytes), the client SHOULD drain its input
buffer before sending the
command frame. This is safe: a server in the file loop sends nothing
unprompted.

### No echo

If no echo arrives after the final retry, the peer is v0.2.1 (or older,
or not SLIDE at all). The client:

- MUST NOT send a command frame (see §6),
- MAY continue with an ordinary file transfer or send `CTRL_FIN`,
- MUST NOT treat this as a session error.

The server never left its file loop, so the session is undamaged.

## §3. Command frames

A command frame is an ordinary v0.2.1 frame:

```
[SOF=0x01] [SEQ=0] [LEN_H] [LEN_L] [PAYLOAD...] [CRC_H] [CRC_L]
```

- `SEQ` is **0**, mirroring the file-header convention. The reply stream
  then starts at 1, exactly as file data does.
- `PAYLOAD[0]` is the opcode. `PAYLOAD[1..]` are its operands.
- CRC is computed and checked exactly as for any other frame. On CRC
  failure the server NAKs seq 0 and the client retransmits, using the
  existing retry logic.

### Opcodes

| Value | Name       | Operands                         |
|-------|------------|----------------------------------|
| 0x00  | —          | reserved, invalid                |
| 0x01  | CMD_VOLS   | none                             |
| 0x02  | CMD_DIR    | drive, user, [match pattern]     |
| 0x03–0x7F | —      | reserved for future amendments   |
| 0x80–0xFF | —      | private / vendor use             |

Opcodes live inside a CRC-protected payload sent only to a confirmed
v0.3 peer, so they are unconstrained by the control-byte allocation.

## §4. Reply streams

The server ACKs the command frame (`ACK 0`), then sends a reply as a
sequence of ordinary data frames with `SEQ` = 1, 2, 3…, terminated by a
zero-length frame — byte-for-byte the same shape as a file transfer. The
client ACKs every `WIN_SIZE` frames and ACKs the terminator, and NAKs on
CRC or sequence error. Sequence numbers wrap mod 256 as for file data.

```
CLIENT                              SERVER
  |  --- ENQ --->                     |
  |  <--- ENQ, '0001' ---             |
  |  --- cmd frame (seq 0) --->       |
  |  <--- ACK 0 ---                   |
  |  <--- status frame (seq 1) ---    |
  |  <--- record frames (seq 2…) ---  |
  |  --- ACK every WIN_SIZE --->      |
  |  <--- zero-length frame ---       |
  |  --- ACK --->                     |
  |                                   |   (server returns to file loop)
```

Reply frames MUST NOT exceed `FRAME_SIZE` (1024) payload bytes.

### The status frame

The first reply frame (`SEQ` 1) is always a **status frame**. Its payload
is at least one byte:

```
[STATUS]
```

| Value | Name          | Meaning                                  |
|-------|---------------|------------------------------------------|
| 0x00  | ST_OK         | Command succeeded; record frames follow  |
| 0x01  | ST_OPCODE     | Unknown or unsupported opcode            |
| 0x02  | ST_OPERAND    | Missing or malformed operand             |
| 0x03  | ST_NODRIVE    | Drive not present or not selectable      |
| 0x04  | ST_IO         | BDOS/BIOS error during enumeration       |
| 0x05  | ST_BUSY       | Server cannot service the request now    |

On any non-zero status the server sends the zero-length terminator
immediately and emits no record frames.

A status frame payload MAY be longer than one byte in a future version.
Clients MUST ignore trailing bytes they do not understand.

`ST_OK` with no record frames (status frame followed straight by the
terminator) is a valid empty result — an empty directory, for instance.
It is distinct from an error.

### Record frames

Frames `SEQ` 2 onward carry fixed-size records, packed with no padding
between them. **A record is never split across a frame boundary**, so
each record frame's payload length is an exact multiple of the record
size. This keeps both the server's emit loop and the client's parser
trivial.

### Ending a session after a command

A client MAY end the session immediately after a command exchange, with
no file transferred at all. `CTRL_FIN` is tested before `SOF` in the
server's file loop (`slide.asm:884`), and the FIN handler has no
precondition about having received anything, so a
handshake → command → FIN session is well formed and exits cleanly.
Nothing is created on the server's disk either: file creation only
happens on the header-frame path.

Ending the session means **sending `CTRL_FIN`**, not merely closing the
port. The server's file loop retries indefinitely on timeout
(`slide.asm:883`) — unlike the handshake, it has no retry budget — so a
client that just disconnects leaves `slide.com` waiting until someone
intervenes at the keyboard. `CTRL_CAN` is an equally valid exit. Silence
is not.

## §5. The commands

### `CMD_VOLS` (0x01)

Command payload:

```
[0x01]
```

Reply: exactly one 5-byte record.

| Offset | Size | Field       | Meaning                                    |
|--------|------|-------------|--------------------------------------------|
| 0      | 2    | present     | LE bitmap, bit n = drive n (0=A) selectable |
| 2      | 2    | logged      | LE bitmap, bit n = drive n logged in       |
| 4      | 1    | current     | current default drive, 0-based             |

Two bitmaps rather than one, because CP/M cannot answer the obvious
question directly. BDOS 24 returns the **login vector** — drives that
have been accessed since the last reset — which is not the same as
drives that exist. `logged` is that vector verbatim. `present` is the
server's best knowledge of what can actually be selected, by whatever
means it has (BIOS drive table, build-time configuration, or trial
select).

A server with no better source than BDOS 24 MAY set `present` equal to
`logged`. Clients SHOULD present `present` to the user and MAY use
`logged` to hint which volumes are already spun up.

### `CMD_DIR` (0x02)

Command payload:

```
[0x02] [DRIVE] [USER] [MATCH (11 bytes, optional)]
```

| Field | Size | Meaning                                                |
|-------|------|--------------------------------------------------------|
| DRIVE | 1    | 0 = A … 15 = P, or 0xFF = current default drive        |
| USER  | 1    | 0–15, or 0xFF = current user number                    |
| MATCH | 11   | Optional 8+3 pattern, space-padded, `?` = wildcard     |

If `LEN` is 3 the match pattern is absent and means match everything
(equivalent to eleven `?` characters). If `LEN` is 14 the pattern is
present. Any other length is `ST_OPERAND`.

The pattern is in CP/M FCB layout — 8 name bytes then 3 type bytes, both
space-padded, uppercase — so a server can copy it straight into a search
FCB with no translation.

Reply: zero or more 16-byte records.

| Offset | Size | Field  | Meaning                                          |
|--------|------|--------|--------------------------------------------------|
| 0      | 1    | user   | user number 0–15                                 |
| 1      | 11   | name   | 8+3, space-padded, uppercase, attribute bits masked off |
| 12     | 1    | attr   | bit0 R/O, bit1 SYS, bit2 archive; others reserved 0 |
| 13     | 3    | size   | file size in 128-byte records, 24-bit LE         |

Notes on the fields:

- **name** is the raw CP/M directory name with the high bits of bytes 8,
  9 and 10 cleared. Those bits are the attribute flags and are reported
  separately in `attr`, so the name is clean ASCII the client can print
  without post-processing.
- **size** is in 128-byte records because that is the only granularity
  CP/M has. It is the value BDOS 35 (`F_FSIZE`) leaves in the FCB's
  random-record field — three bytes, little-endian, no conversion
  needed. 24 bits covers 2 GB of records, far beyond any CP/M volume.
  Clients wanting bytes multiply by 128 and should present the result as
  an upper bound, exactly as they already do for the file-transfer
  header's size field.
- Formatting is entirely the client's job. The server emits records.

At 16 bytes per record, a 1024-byte frame carries 64 entries.

**Extent coalescing.** BDOS search (functions 17/18) returns one
directory entry per *extent*, so a file larger than 16 KB appears more
than once. A server MUST report each file exactly once — the
straightforward approach is to skip entries whose extent number is
non-zero, and to obtain the true size with `F_FSIZE` per file. That is
one extra BDOS call per file, which is why a large directory takes
noticeable time to enumerate; clients SHOULD allow for it in their
timeouts.

## §6. The one prohibition

**A client MUST NOT send a command frame to a peer that has not echoed
`CTRL_ENQ`.**

This is not a style preference. A v0.2.1 server fails in two distinct
ways if given an unannounced frame:

1. **It creates a junk file.** The file loop (`slide.asm:881`) treats any
   `SOF` as a file header. It will run `parse_filename` over the command
   payload and hand the result to `create_file`. Whatever lands on the
   disk is unpredictable.

2. **It can desynchronise.** Using some other start byte to avoid (1)
   makes it worse: the v0.2.1 peer skips the unrecognised byte, then
   resynchronises on the first payload or CRC byte that happens to be
   `0x01`. `recv_frame.after_sof` reads `rx_len` and trusts it with no
   bound (`slide.asm:559`), so the peer will attempt to read a multi-KB
   payload into `RXBUF` from a length it invented.

Single control bytes are safe to send blind; frames are not. The probe
exists precisely so that no frame is ever sent blind.

## §7. Cancel

`CTRL_CAN` retains its v0.2.1 §2 meaning and remains valid at every
point in a command exchange where a raw byte is read — in particular
while the server waits for a command frame after echoing `CTRL_ENQ`, and
in the client's wait for the echo.

Because v0.2.1 handles CAN in every wait loop (`slide.asm:886`, `:518`,
`:464`, and the `slide-py` equivalents), a client whose probe or command
hangs always has a deterministic way to return both sides to idle: send
CAN, take the echo, drop the session.

## §8. Compatibility summary

| Client  | Server  | Behaviour                                                   |
|---------|---------|-------------------------------------------------------------|
| v0.3    | v0.3    | Full command support.                                       |
| v0.3    | v0.2.1  | Probe ignored; no echo; client falls back to file transfer or FIN. Server never leaves its file loop. Session undamaged. |
| v0.2.1  | v0.3    | Client never sends `CTRL_ENQ`; server never enters command mode. Behaviour bit-identical to v0.2.1. |

### Verification status

The `v0.3 client → v0.2.1 server` row is **measured, not inferred**. The
reference probe (`slide probe`, `slide-rs`) was run against `slide.com`
v0.5.2 on MicroBeast hardware at 19200 baud on 2026-08-07 and
2026-08-08. Confirmed:

- **`CTRL_ENQ` is inert.** Three attempts drew no echo and **no bytes
  whatsoever** — the server emitted nothing at all, not merely nothing
  recognisable.
- **The probe leaves no residue.** A `CTRL_FIN` sent straight afterwards
  was echoed normally, so the server was still in its file loop and
  exited cleanly.
- **Fall-back to ordinary file transfer works.** A 2845-byte file sent
  immediately after an ignored probe transferred and was ACKed normally,
  followed by a clean FIN exchange. Three skipped `CTRL_ENQ` bytes leave
  no state that disturbs a following header frame.
- **The §4 zero-file session is well formed.** handshake → probe → FIN
  with nothing transferred exits cleanly.
- **Probing after a completed file transfer behaves identically.** A
  second probe issued once the transfer's EOF frame had been ACKed drew
  the same silence, and the wire was measurably idle going in — nothing
  was queued ahead of it. In receive mode the preceding transfer's ACKs
  flow *from* the server, so this confirms the client had consumed them
  all and that the position is as clean as the post-handshake one.
- **The v0.2.1 §1 wakeup signature arrives intact** when the client
  starts the session itself (see below).

That covers every probe position §2 permits except "after a completed
command exchange", which cannot be tested until a v0.3 server exists.

Two limits on how far to read the residue result. It was measured on a
clean transfer with no NAK and no retransmission, so it says nothing
about the noisy recovery paths. And it was measured with the *client*
sending frames; the §9 requirement to drain after a reply stream
concerns the opposite direction, where the server sends and the client
ACKs, which no implementation exercises yet. Neither is grounds for
relaxing that requirement.

### The wakeup signature is not a capability signal

The wakeup signature (v0.2.1 §1) MUST NOT be used to detect command
support. It carries no version, and a host that attaches to an already
running `SLIDE R` has simply missed it.

Its reliability also depends on who starts the session, which is worth
stating precisely because it constrains client design:

- **Host attaches to a Z80 already in SLIDE mode** — the signature is
  unreliable and may be absent entirely. `send_wakeup` transmits via
  `uart_tx`, which auto-flow-control blocks while CTS is low, so if the
  host was not asserting RTS when the user started `SLIDE R`, at most the
  leading `0x1B` reaches the wire and the remaining six bytes time out
  silently.
- **Host starts the session itself** — the signature arrives intact,
  because the host has held RTS from before the Z80 entered SLIDE mode.
  Confirmed on hardware in the run described above.

On a MicroBeast the second case is the only practical one anyway: the
CP/M console *is* the SLIDE UART, so a host tool that opens the port
takes away the terminal the operator would have used to type `SLIDE R`.
Clients should type the command at the CP/M prompt themselves (see §9),
which incidentally makes the signature dependable as a liveness and
mode-entry confirmation — just never as a capability or version check.

## §9. Implementation notes

### Z80 (`slide.com`)

- One extra branch in `.file_loop` (`slide.asm:881`), testing `CTRL_ENQ`
  alongside the existing `CTRL_FIN` / `CTRL_CAN` / `SOF` tests. No new
  mode and no command-line change: `SLIDE R` becomes the server.
- On `CTRL_ENQ`: echo it, send the four `VER` bytes from a static
  `DB '0001'`, then wait for `SOF` as the file loop already does —
  falling back to the file loop if `FIN` or `CAN` arrives instead. The
  reply is unconditional and carries no state, so this is five `uart_tx`
  calls and nothing else.
- The command frame is received into `RXBUF` like any other, but is at
  most 14 bytes. Copy the opcode and operands to a small static area
  before building the reply, so `RXBUF` is free for the outgoing record
  batch.
- A command exchange MUST leave the file-receive state
  (`expected_seq`, `buf_ptr`, `buf_used`, `retry_count`) as the file loop
  expects it — either untouched, or reset the same way as the per-file
  reset block at `slide.asm:917`.
- Emit whole records only. Fill up to 64 records, send the frame, repeat.
- `CMD_DIR` needs a second FCB and its own DMA buffer for the search, or
  careful save/restore of the default DMA address at `0x0080` — the
  command tail lives there and is no longer needed by then, but
  `create_file` and friends assume the standard DMA setup.

#### Draining after a reply stream (required)

A server MUST `CALL uart_flush_rx` when a command reply completes, before
returning to the file loop.

This closes a hazard the command channel creates. In v0.2.1 receive mode
the Z80 never sends frames, so it never consumes ACKs, so no ACK can ever
be sitting unread when `.file_loop` next reads a byte. A reply stream
changes that: if the exchange hit a NAK or a retransmit, the client may
have queued more ACKs than the server's send loop consumed.

The leftovers are not harmless. `.file_loop` skips a stray `CTRL_ACK`
(`0x06`) because it matches none of its tests — and then reads the
**sequence byte** behind it, which is unconstrained. A queued `ACK 1`
leaves `0x01` next in line, which reads as `SOF`, and the server starts
parsing a frame out of residue with `rx_len` taken from whatever follows.
A queued `ACK 4` reads as `FIN` and ends the session early. Neither
failure is detectable by CRC, because the frame the server thinks it is
reading never existed.

One flush call removes the whole class. The same argument applies to the
existing send path, where it is noted as optional hardening
(`docs/IDEAS.md`); here it is required, because the reply stream is what
puts ACKs in flight during receive mode in the first place.

### PC (`slide-py`, `slide-rs`)

- `slide-py`'s frame reader lives in `recv.py` (`_recv_frame_after_sof`).
  Reading reply streams from a sending client means hoisting it into
  `common.py`. This is a client refactor, not a protocol change.
- The command channel is most useful in a combined client that can both
  send and receive; the split `slide-send` / `slide-recv` entry points do
  not naturally host it.
- Clients SHOULD cache the probe result per session — but MUST still
  send `CTRL_ENQ` before each command frame, since it is the server's
  only signal that the next frame is not a file header.

#### Starting the server when the console is the same UART

On a MicroBeast the CP/M console *is* the SLIDE UART, so there is no
moment at which both a terminal emulator and a SLIDE client can hold the
line. "Start the client, then type `SLIDE R`" is not a workflow the
operator can physically perform: opening the port removes the terminal
they would type into.

Clients SHOULD therefore be able to type the start command themselves.
CP/M reads its command line from the console, so writing the text to the
port is indistinguishable from typing it:

1. Write a bare `CR` to clear any half-typed line and draw a fresh
   prompt.
2. Wait ~250 ms, then discard whatever that echoed.
3. Write the command (`SLIDE R`) followed by `CR`.
4. Proceed to the RDY handshake.

This also satisfies the v0.2.1 §1 RTS ordering requirement for free: the
port is open, and RTS asserted, before the Z80 enters SLIDE mode.

Two consequences for the handshake that follows. The client will see the
command echo, the CP/M banner and the wakeup signature — several dozen
stray bytes — before the peer's RDY, so the handshake MUST drain
everything available in each listening window rather than testing a
single byte per retry. And the first RDY may be written while CP/M is
still loading `SLIDE.COM` from disk, so it can be lost to the UART
initialisation in `slide.com`; the client's existing RDY retry covers
this, but a client that sends RDY only once will hang.

## §10. Worked example

Client asks a Z80 for its volumes. Bytes on the wire, hex:

```
C→Z  05                          ENQ
Z→C  05 30 30 30 31              ENQ echo, command-set version '0001'
C→Z  01 00 00 01 01 <crc_h crc_l>    SOF, seq 0, len 1, opcode CMD_VOLS
Z→C  06 00                       ACK 0
Z→C  01 01 00 06 00 <5 bytes> <crc>  SOF, seq 1, len 6: status + record
Z→C  01 02 00 00 <crc>           SOF, seq 2, len 0: terminator
C→Z  06 02                       ACK 2
```

Here the status byte and the single 5-byte `CMD_VOLS` record share one
frame — a status frame MAY carry the reply's records when they fit,
provided the total stays within `FRAME_SIZE`. For `CMD_DIR`, where the
record count is not known up front, servers will normally send the status
frame alone and stream records behind it.

## §11. Versioning

- Wire protocol version: **v0.3**.
- Command-set version (the `CTRL_ENQ` echo's `VER` field): **`'0001'`**.
- Reference Z80 application: `slide.com` v0.6 (first version
  implementing wire v0.3).

The command-set version and the wire version are deliberately separate
numbers. Adding an opcode bumps the command-set minor and nothing else —
it is not a wire change, and a v0.3 client and a v0.3 server with
different minors interoperate on the opcodes they share. Only a change
to framing, control bytes or session flow bumps the wire version.
- Compatibility floor: wire v0.2.1 / `slide.com` v0.5.0.

## §12. Cross-references

- [SPEC-v0.2.1.md](SPEC-v0.2.1.md) — wakeup signature and bidirectional
  cancel; the version this amends.
- [SPEC-v0.2.md](SPEC-v0.2.md) — base bidirectional and multi-file
  specification.
- [SLIDE_Z80_REQUIREMENT.md](SLIDE_Z80_REQUIREMENT.md) — host-side
  consumer requirements.
