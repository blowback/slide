# SLIDE v0.2 Specification — Bidirectional & Multi-File Transfer

## Overview

SLIDE v0.1 supports PC→Z80 file transfer only. v0.2 adds:
1. **Bidirectional transfer** — Z80 can send files to the PC
2. **Multi-file support** — multiple files transferred in a single session
3. **New PC-side `slide-recv` command** — receives files from Z80

## Protocol Changes

### New Control Byte

| Byte | Name     | Meaning                        |
|------|----------|--------------------------------|
| 0x04 | CTRL_FIN | Session complete, no more files |

Existing control bytes unchanged:
- 0x06 ACK, 0x15 NAK, 0x11 RDY

### Session Flow — Multi-File

The single-file header→data→EOF sequence is now repeated per file, wrapped in a session:

```
SENDER                          RECEIVER
  |                                |
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
  |  <--- ACK ---                  |
  |  --- data frames ----------->  |
  |  --- EOF frame (len=0) ------> |
  |  <--- ACK ---                  |
  |                                |
  |  --- FIN --->                  |   (no more files)
  |  <--- FIN ---                  |   (acknowledged)
  |                                |
```

### Startup Handshake — Direction Negotiation

Both sides exchange RDY. The sender transmits first, the receiver responds:

**PC sending to Z80 (`slide-send` + `SLIDE R`):**
1. PC sends RDY
2. Z80 (in receive mode) sends RDY back
3. PC begins sending header for first file

**Z80 sending to PC (`SLIDE S file1.com file2.com` + `slide-recv`):**
1. Z80 sends RDY
2. PC (in receive mode) sends RDY back
3. Z80 begins sending header for first file

This is symmetric — whoever is the sender sends RDY first, the receiver echoes it to confirm. No direction byte needed; each side already knows its role from the command line.

### FIN Handshake

After the last file's EOF frame is ACK'd:
1. Sender sends CTRL_FIN
2. Receiver sends CTRL_FIN back
3. Both sides exit cleanly

If the receiver gets a header frame instead of FIN after an EOF ACK, it knows another file is coming.

### Frame Format

Unchanged from v0.1:
```
[SOF=0x01] [SEQ] [LEN_H] [LEN_L] [PAYLOAD...] [CRC_H] [CRC_L]
```

Sequence numbers reset to 0 for each file's header frame, then 1, 2, 3... for data frames.

## Z80 Implementation — `SLIDE.COM`

### Command Line Syntax

```
SLIDE R                          — receive file(s) from PC
SLIDE S MYFILE.COM               — send one file to PC  
SLIDE S FILE1.COM FILE2.TXT      — send multiple files to PC
SLIDE S *.COM                    — send with wildcards (if CP/M shell expands)
```

CP/M puts the command tail at 0x0080 (length byte) and 0x0081+ (text). Parse this to determine mode and filenames.

Note: CP/M's FCB at 0x005C only holds the first filename. For multiple files, parse the command tail directly. For wildcard expansion, use BDOS function 17 (Search for First) and 18 (Search for Next).

### New Z80 Routines Needed

#### `send_frame` — Build and transmit a frame
```
; Entry: HL = payload buffer, BC = payload length, A = sequence number
; Sends: SOF, SEQ, LEN_H, LEN_L, payload bytes, CRC_H, CRC_L
; Uses: crc_update_a for CRC computation over SEQ+LEN+PAYLOAD
```

This is the mirror of `recv_frame`. For each byte sent, update the running CRC, then send the two CRC bytes at the end.

#### `recv_control` — Wait for ACK/NAK from PC
```
; Returns: A = control byte (ACK/NAK/FIN), B = sequence number (if ACK/NAK)
;          carry set on timeout
```

#### `send_file` — Main send loop for one file
```
; Entry: FCB set up with filename
; Opens file, reads into buffer, frames and sends with sliding window
```

Logic:
1. Open file via BDOS F_OPEN
2. Loop: read up to WIN_SIZE * FRAME_SIZE bytes into RXBUF (reuse the same buffer)
   - Use BDOS F_READ (function 20) to read 128-byte records
3. Frame each FRAME_SIZE chunk, send with incrementing SEQ
4. After WIN_SIZE frames (or end of data), wait for ACK/NAK
5. On NAK, rewind buffer position and retransmit from requested SEQ
6. On ACK, continue with next window
7. After all data sent, send zero-length EOF frame, wait for ACK

#### `read_from_disk` — Fill buffer from file
```
; Fills RXBUF with up to FLUSH_SIZE bytes from the open file
; Returns: BC = bytes actually read (may be less at end of file)
;          carry set if no more data (EOF)
```

Uses BDOS F_SETDMA + F_READ in a loop, 128 bytes at a time.

### Modified Z80 Routines

#### `entry` — Parse command line
- Read byte at 0x0080 for command tail length
- Parse first token after spaces: 'R' or 'S'
- If 'S': parse subsequent filename(s), call `send_session`
- If 'R': call `recv_session`

#### `recv_session` — Multi-file receive (wraps existing `recv_file`)
```
send RDY
wait for RDY from sender
loop:
    wait for next byte — if FIN, send FIN back, exit
    if SOF, it's a header frame — receive it
    create file
    ACK header
    call recv_file (existing code)
    goto loop
```

The key change: after `recv_file` completes (EOF frame ACK'd), don't exit — peek at the next incoming byte. If it's SOF, another file is coming. If it's CTRL_FIN, we're done.

#### `send_session` — Multi-file send
```
send RDY
wait for RDY from receiver
for each filename:
    set up FCB
    open file
    send header frame (filename + filesize)
    wait for ACK
    call send_file
    close file
send FIN
wait for FIN
exit
```

### Getting File Size on Z80

CP/M doesn't directly give you a file size in bytes. Options:
- Use BDOS function 35 (Compute File Size) which returns the number of 128-byte records in the FCB's random record field (r0, r1, r2 at FCB+33/34/35)
- Multiply by 128 to get size in bytes
- This will be slightly larger than the actual file (padded to 128-byte boundary) but that's fine — CP/M files are always record-aligned anyway

## PC Implementation

### `slide-send` Changes

The existing `slide-send` already handles single files. Changes:
- Accept multiple filenames: `slide-send /dev/ttyUSB0 file1.com file2.txt file3.hex`
- After each file's EOF ACK, send the next file's header (or FIN if done)
- Updated startup: send RDY, wait for RDY echo

### New `slide-recv` Command

```bash
uv run slide-recv /dev/ttyUSB0 [--baud 19200] [--output-dir ./downloads]
```

Receives one or more files from Z80. Logic:
1. Wait for RDY from Z80
2. Send RDY back
3. Loop:
   a. Read next byte — if FIN, send FIN, exit
   b. If SOF, receive header frame, extract filename and size
   c. Open local file for writing
   d. Receive data frames, validate CRC, write to file
   e. ACK every WIN_SIZE frames, NAK on error
   f. On EOF frame, ACK, close file, loop

### Project Structure Update

```
pc/
├── pyproject.toml
└── slide/
    ├── __init__.py
    ├── common.py      ← shared: CRC, frame building/parsing, constants
    ├── send.py         ← slide-send (PC→Z80, multi-file)
    └── recv.py         ← slide-recv (Z80→PC, multi-file)
```

Extract shared code (CRC, `build_frame`, `recv_control`, constants) into `common.py`.

### pyproject.toml Update

```toml
[project.scripts]
slide-send = "slide.send:main"
slide-recv = "slide.recv:main"
```

## Implementation Priority

1. **Refactor PC-side** — extract `common.py`, update `send.py` for multi-file + new handshake
2. **Write `recv.py`** — PC receive mode
3. **Z80 send mode** — `send_frame`, `recv_control`, `send_file`, `send_session`
4. **Z80 command parsing** — parse R/S and filenames from command tail
5. **Z80 multi-file receive** — wrap existing recv in session loop
6. **Test single file Z80→PC** first, then multi-file both directions

## Test Plan

1. Single file PC→Z80: `slide-send /dev/ttyUSB0 test.txt` / `SLIDE R`
2. Single file Z80→PC: `SLIDE S TEST.TXT` / `slide-recv /dev/ttyUSB0`
3. Multi-file PC→Z80: `slide-send /dev/ttyUSB0 a.txt b.txt c.txt` / `SLIDE R`
4. Multi-file Z80→PC: `SLIDE S A.TXT B.TXT C.TXT` / `slide-recv /dev/ttyUSB0`
5. Verify received files match originals (compare checksums)
6. Error recovery: unplug/replug USB mid-transfer, verify NAK/retransmit works
