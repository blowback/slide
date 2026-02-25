# SLIDE - Serial Line Inter-Device (file) Exchange

File transfer from PC to MicroBeast (Z80! CP/M!) over a serial link. Sliding window protocol with CRC-16 error detection and hardware flow control.

95%+ link utilisation for files > 1K. CP/M binary is 1.3 KBytes.

Other Z80 based computers are available, and might work with a bit of fiddling about. IO ports and baud rates and such.

## BEAST side

Build with sjasmplus:

```
make slide.com
```

Copy `slide.com` to a CP/M disk and run:

```
A> SLIDE
```

Or, you can `make disk` (you'll need cpmtools) and transfer  `slide_p25.img` to your system using whchever inferior serial transfer tools you are currently having to tolerate.

SLIDE waits up to ~30 seconds for the PC to connect.

## PC side

Requires Python 3.10+ and [uv](https://docs.astral.sh/uv/):

```
cd slide-pc
uv sync
uv run slide-send /dev/ttyUSB0 myfile.com
```

Options:

```
uv run slide-send /dev/ttyUSB0 myfile.com --baud 19200 --debug
```

- `--baud` — baud rate (default: 19200)
- `--debug` — show wire-level frame and control byte trace

If you change the baudrate, you'll have to change the baudrate divisor in `slide.asm` to match

## Protocol

- 19200 baud, 8N1, RTS/CTS hardware flow control
- Sliding window: 4 frames, 1024 bytes/frame
- CRC-16-CCITT (poly 0x1021, init 0xFFFF)
- Frame: `[SOF 0x01] [SEQ] [LEN_H] [LEN_L] [PAYLOAD...] [CRC_H] [CRC_L]`
- Control: `ACK 0x06 + seq`, `NAK 0x15 + seq`, `RDY 0x11`, `CAN 0x18`

## Hardware

- Z80 at 8MHz with TL16C550 UART (1.8432MHz crystal, divisor 6 = 19200 baud)
- USB serial cable on PC side
- UART FIFOs enabled with auto RTS/CTS flow control

## Things I've tested

YMMV, but I've tried:

- sending a zero-length file
- sending files that fit in a single packet (<1 Kb)
- sending files that require multiple packets
- sending files that are bigger than RAM
- sending files that are bigger than free disk space (and handling the error)
- disconnecting PC part way thru transfer
- disconnecting Z80 part way thru transfer
- overwriting existing files on z80 disk
- starting z80 before pc
- starting pc before z80
- sending filename with no extension
- sending very long filenames on PC side
- writing to A: (a bug in 1.7 makes this weirder than it ought to be)

## Things I've not tested

- target file exists and is read-only (need to figure out the `stat` runes)
