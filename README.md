# SLIDE - Serial Line Inter-Device (file) Exchange

File transfer from PC to the [FeerSum Beasts MicroBeast Z80 Computer](https://feersumbeasts.com/microbeast.html) (Z80! CP/M!) over a serial link.

Sliding window protocol with CRC-16 error detection and hardware flow control.

95%+ link utilisation for files > 2K. CP/M binary is 2.5 KBytes.

Other Z80 based computers are available, and might work with a bit of fiddling about. IO ports and baud rates and such.

![Transfer example](images/transfer.png)

## Installation

### On the MicroBeast

Copy `slide.com` from [the latest release](https://github.com/blowback/slide/releases) to your MicroBeast, using whichever serial transfer software you are currently forced to tolerate.

Alternatively you can use the monitor's Y-Modem transfer capability to copy over the disk image `slide_p25.img` using the "Address from file" option, then when it has transferred select the "CP/M disk" option.

Once you've got on your MicroBeast, it's a good idea to copy it on to your RAM disk and use the MicroBeast's `save` utility to persist it.

### On your PC

Copy the relevant binary for your system (one of `slide-linux-amd64`, `slide-macos-amd64`, or `slide-windows-amd64.exe`) from the [latest release](https://github.com/blowback/slide/releases) to your PC. These are rust executables that handle both sending and receiving. If you prefer to use python, the original scripts are in `slide-py` - see [Python scripts](#python-scripts) for more info.

#### A note for MacOS users

Apple have made it impossible to "staple" single Mach-O binaries like `slide`: this means that the first time you run it, Gatekeeper on your system will do an online check, so you need internet access.

If you want to run it offline, you can:

```
xattr -d com.apple.quarantine ./slide-macos-amd64
```

which will disable GateKeeper checks entirely. 


Don't worry, we are fully legit ;-) If you are nervous about this, you can verify the binary on a different, internet-connected mac:

```
codesign -dv --verbose=4 ./slide-macos-amd64

```

which will show you my team credentials, and:

```
codesign --verify --verbose ./slide-macos-amd64
```

should get you `valid on disk` and `satisfies its Designated Requirement`.

You can then check the notarization status with:

```
spctl --assess --verbose ./slide-macos-amd64
```

which should say `accepted` and `source: Notarized Developer ID`.

If your happy with that, copy that exact same binary to your air-gapped mac. 

## Transferring files

### Sending files from PC to MicroBeast

On the MicroBeast, run:

```
slide
```

`slide` is an alias for `slide r` - slide in receive mode. This will wait up to 30 seconds for the PC to establish a link.

On the PC, run:

```
slide send /dev/ttyUSB0 TEST1K.dat
```

To kick off a transfer.

You can send multiple files in one go:

```
slide send /dev/ttyUSB0 TEST1K.dat TEST2K.dat TEST4K.dat
```

### Sending files from MicroBeast to PC 

It works in reverse too: this time we use "send" mode on the MicroBeast:

```
send S TEST1K.DAT 
```

and on the PC side:

```
slide recv /dev/ttyUSB0
```

This will put received files in the current directory. You can change that:

```
slide recv --output-dir /tmp /dev/ttyUSB0
```





## Build it yourself

If you want to build it yourself, the top-level `Makefile` will build the z80 binary and a CP/M disk image. You'll need [sjasmplus](https://github.com/z00m128/sjasmplus) and [cpmtools](https://github.com/z00m128/sjasmplus).

Python PC tools are in `slide-py`.

Rust PC tools are in `slide-rs` - just `cargo build --release` in there.






### Python scripts

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
- USB serial port on PC side
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
- linux, windows and macos builds of the rust tool 
- very light testing on macos and windows 


## Things I've not tested

- target file exists and is read-only (need to figure out the `stat` runes)
