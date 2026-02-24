# SLIDE - Serial Line Inter-Device (file) Exchange

## CONTEXT FOR CLAUDE CODE

This is a custom file transfer protocol for sending files from a modern PC to a Z80 microcomputer running CP/M, over a serial link. The name "SLIDE" is both an acronym and a verb — you "slide" files over to the Z80.

### Hardware

- **Z80**: 8MHz, custom/modern retro computer build
- **UART**: TL16C550 at I/O base address **0x20**, 16-byte FIFO, with auto flow control (AFE) support
- **Crystal**: 1.8432MHz feeding the TL16C550 (divisor 6 = 19200 baud)
- **Link**: USB serial cable, 19200 baud, 8N1, RTS/CTS hardware flow control
- **Console**: Separate keyboard and LED display (NOT on the UART). IOBYTE set to 0b01010110 to keep CP/M console off the UART
- **Assembler**: sjasmplus
- **Hex format convention**: Use `0xXXXX` style
- **Instruction case**: UPPERCASE mnemonics

### Protocol Design

```
Frame: [SOF=0x01] [SEQ] [LEN_H] [LEN_L] [PAYLOAD...] [CRC_H] [CRC_L]
Control (Z80→PC): [ACK 0x06] [SEQ] | [NAK 0x15] [SEQ] | [RDY 0x11]
```

- CRC-16-CCITT (poly 0x1021, init 0xFFFF) over SEQ+LEN+PAYLOAD
- Sliding window of 4 frames, 1024-byte max payload per frame
- 4KB receive buffer in RAM, flushed to disk via CP/M BDOS when full
- Auto RTS/CTS flow control handles pauses during disk I/O
- Startup handshake: Z80 sends RDY, PC waits for RDY before sending header

### Session Flow

1. Z80 sends RDY byte when ready
2. PC sends header frame (seq 0): null-terminated filename + 4-byte LE filesize
3. Z80 ACKs header, creates file on disk
4. PC streams data frames (seq 1, 2, 3...) in sliding window
5. Z80 ACKs every WIN_SIZE frames, NAKs on CRC error or timeout
6. PC sends zero-length frame to signal end of transfer
7. Z80 flushes remaining buffer, ACKs, closes file

### Project Structure

```
slide-project/
├── CONTEXT.md          ← this file
│   ├── slide.asm       ← main Z80 receive program (sjasmplus)
│   ├── slidediag.asm   ← UART echo diagnostic tool
│   └── slidediag_pc.py ← PC-side diagnostic (standalone, no deps)
└── slide-pc/
    ├── pyproject.toml   ← uv project with pyserial dependency
    └── slide/
        ├── __init__.py
        └── send.py      ← PC-side sender script
```

### PC-side Usage

```bash
cd pc
uv sync
uv run slide-send /dev/ttyUSB0 myfile.com --debug
```

### Current Status: DATA FRAME CRC MISMATCH

**What works:**

- UART link verified working via slidediag (echo test passes, correct bytes, correct baud rate)
- Startup handshake (Z80 sends RDY, PC receives it)
- Header frame received and CRC validated successfully on Z80
- Header ACK sent and received by PC
- File creation on Z80 works

**What's broken:**

- Every data frame gets NAK'd by the Z80
- We just added debug output to distinguish timeout vs CRC mismatch
- The Z80 will now print one of:
  - `DBG: timeout in payload` — bytes stopped mid-frame
  - `DBG: CRC mismatch got/exp: XXXX YYYY` — CRC computed vs received
- We haven't yet seen this debug output — that's the immediate next step

**Bugs found and fixed so far:**

1. CRC lookup table byte order was swapped (stored low,high but update routine expected high,low)
2. `crc_update_a` trashes HL, which was being used as the buffer pointer in `recv_frame` — fixed by saving/restoring via `frame_dst` memory variable
3. `expected_seq` in `recv_file` was initialised to 0, but data frames start at seq 1 (seq 0 is the header)
4. UART base address was 0x80 in code but hardware is at 0x20
5. No startup handshake — PC could send before Z80 was ready. Fixed with RDY signal
6. PC sent data immediately after header ACK while Z80 was doing disk I/O for file creation — added 0.5s delay + input buffer flush

**Key suspicion for current bug:**
The header frame (14 bytes payload) works fine. Data frames (384 bytes payload for the test file) fail. The difference is the long payload receive loop. Possible causes:

- CRC accumulation bug that only manifests over many bytes
- The HL save/restore around `crc_update_a` in the payload loop might still have a subtle issue
- Some interaction between `uart_rx_timeout` and the payload loop

**Immediate next step:**
Run the current code with `--debug` on PC side and read the Z80's display to see if it's "timeout in payload" or "CRC mismatch got/exp: XXXX YYYY". This will pinpoint the problem.
