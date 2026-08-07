# SLIDE - Serial Line Inter-Device (file) Exchange

A custom file transfer protocol for Z80/CP/M systems over serial.

## Setup

```bash
uv sync
```

## Usage

```bash
# Via the installed command
uv run slide-send /dev/ttyUSB0 myfile.com

# Or directly
uv run python -m slide.send /dev/ttyUSB0 myfile.com

# Custom baud rate
uv run slide-send /dev/ttyUSB0 myfile.com --baud 9600
```
