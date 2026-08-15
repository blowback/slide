import { createWriteStream as fsCreateWriteStream } from "node:fs";
import { SerialPort } from "serialport";

export const SOF = 0x01;
export const CTRL_ACK = 0x06;
export const CTRL_NAK = 0x15;
export const CTRL_RDY = 0x11;
export const CTRL_FIN = 0x04;
export const CTRL_CAN = 0x18;

export const WIN_SIZE = 4;
export const FRAME_SIZE = 1024;

export const WAKEUP_SIG = [0x1b, 0x5e, 0x53, 0x4c, 0x49, 0x44, 0x45];

// v0.2.2: the Z80's BIOS CONOUT path strips bit 7 of every byte it
// transmits (confirmed with diagcpm.com), so slidecpm.asm escapes any byte
// that wouldn't survive it before sending — see SPEC-v0.2.2.md. These must
// match slidecpm.asm's ESC_HIGH/ESC_LIT exactly. The PC's own writes don't
// need escaping (Node's serialport is 8-bit clean end to end), only bytes
// arriving *from* the Z80 need decoding.
const ESC_HIGH = 0x7e; // introduces "next byte | 0x80"
const ESC_LIT = 0x7d; // introduces "next byte, taken literally"

// v0.2.2 §2: one capability byte follows the wakeup signature. The literal
// character '7' means the peer escapes transmitted protocol bytes per the
// scheme above - only then should we unescape what we read from it. A
// human-readable marker rather than a small binary value, so it reads as
// plain text right after "...SLIDE" in a trace dump instead of looking
// like it could be SOF/CTRL_FIN/etc. slide.com (raw UART, always 8-bit
// clean) and pre-v0.2.2 slidecpm.com builds never send this byte, so a
// peer that doesn't escape is the correct default until proven otherwise -
// unescaping unconditionally would corrupt a literal 0x7D/0x7E data byte
// from a peer that was never escaping in the first place.
const WAKEUP_CAP_ESCAPED = 0x37; // '7'

export type Control =
  | { kind: "ack"; seq: number }
  | { kind: "nak"; seq: number }
  | { kind: "rdy" }
  | { kind: "fin" }
  | { kind: "can" };

export type FrameResult =
  | { kind: "data"; seq: number; payload: Buffer }
  | { kind: "fin" }
  | { kind: "cancel" };

export function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export function crc16Ccitt(data: Uint8Array): number {
  let crc = 0xffff;
  for (const byte of data) {
    crc ^= byte << 8;
    for (let i = 0; i < 8; i += 1) {
      if (crc & 0x8000) {
        crc = ((crc << 1) ^ 0x1021) & 0xffff;
      } else {
        crc = (crc << 1) & 0xffff;
      }
    }
  }
  return crc;
}

export function buildFrame(seq: number, payload: Uint8Array): Buffer {
  const length = payload.length;
  const lenH = (length >> 8) & 0xff;
  const lenL = length & 0xff;

  const crcData = Buffer.alloc(3 + length);
  crcData[0] = seq & 0xff;
  crcData[1] = lenH;
  crcData[2] = lenL;
  Buffer.from(payload).copy(crcData, 3);
  const crc = crc16Ccitt(crcData);

  const frame = Buffer.alloc(1 + 1 + 2 + length + 2);
  let off = 0;
  frame[off++] = SOF;
  frame[off++] = seq & 0xff;
  frame[off++] = lenH;
  frame[off++] = lenL;
  Buffer.from(payload).copy(frame, off);
  off += length;
  frame[off++] = (crc >> 8) & 0xff;
  frame[off] = crc & 0xff;
  return frame;
}

export function buildHeaderFrame(filename: string, filesize: number): Buffer {
  const upper = filename
    .split(/[\\/]/)
    .pop()
    ?.toUpperCase() ?? filename.toUpperCase();
  const nameBytes = Buffer.from(upper, "utf8");
  const payload = Buffer.alloc(nameBytes.length + 1 + 4);
  nameBytes.copy(payload, 0);
  payload[nameBytes.length] = 0;
  payload.writeUInt32LE(filesize >>> 0, nameBytes.length + 1);
  return buildFrame(0, payload);
}

export type TraceFn = (dir: "TX" | "RX", bytes: Buffer) => void;

export function formatHexDump(data: Buffer): string {
  const hex = data.toString("hex").match(/.{1,2}/g)?.join(" ") ?? "";
  const ascii = data.toString("latin1").replace(/[^\x20-\x7e]/g, ".");
  return `(${data.length.toString().padStart(4, " ")}) ${hex}  |${ascii}|`;
}

export function makeTraceLogger(filePath: string): TraceFn {
  const stream = fsCreateWriteStream(filePath, { flags: "a" });
  const start = Date.now();
  return (dir, bytes) => {
    const t = ((Date.now() - start) / 1000).toFixed(3).padStart(8, " ");
    stream.write(`[${t}s] ${dir} ${formatHexDump(Buffer.from(bytes))}\n`);
  };
}

function combineTrace(a?: TraceFn, b?: TraceFn): TraceFn | undefined {
  if (!a) {
    return b;
  }
  if (!b) {
    return a;
  }
  return (dir, bytes) => {
    a(dir, bytes);
    b(dir, bytes);
  };
}

export class TransferCancelledError extends Error {
  constructor(filename: string) {
    super(`Transfer cancelled: ${filename}`);
    this.name = "TransferCancelledError";
  }
}

export async function openSerial(
  path: string,
  baud: number,
  debug: boolean,
  fileTrace?: TraceFn
): Promise<SerialSession> {
  const port = new SerialPort({
    path,
    baudRate: baud,
    dataBits: 8,
    stopBits: 1,
    parity: "none",
    rtscts: true,
    autoOpen: false
  });

  await new Promise<void>((resolve, reject) => {
    port.open((err) => {
      if (err) {
        reject(err);
        return;
      }
      resolve();
    });
  });

  // --debug logs every raw byte written/received as a timestamped hex
  // dump on stderr, independent of --trace (which logs the same bytes to a
  // file) - lets a failing transfer be compared byte-for-byte against what
  // actually went out on the wire, not just the parsed frames/errors.
  const consoleTrace: TraceFn | undefined = debug
    ? (dir, bytes) => console.error(`[${new Date().toISOString()}] ${dir} ${formatHexDump(Buffer.from(bytes))}`)
    : undefined;

  return new SerialSession(port, combineTrace(consoleTrace, fileTrace));
}

export class SerialSession {
  private readonly port: SerialPort;

  private readonly queue: number[] = [];

  private readonly waiters: Array<(value: number | undefined) => void> = [];

  private readonly trace?: TraceFn;

  private escapeState: "normal" | "lit" | "high" = "normal";

  // Off by default - only a peer that actually announces v0.2.2 escaping
  // (via the capability byte after its wakeup signature) gets unescaped.
  // See trackWakeupCapability below.
  private escapeDecodingEnabled = false;

  private wakeupMatchLen = 0;

  private awaitingCapabilityByte = false;

  constructor(port: SerialPort, trace?: TraceFn) {
    this.port = port;
    this.trace = trace;
    this.port.on("data", (chunk: Buffer) => {
      // Trace the raw wire bytes (still escaped, if the peer is escaping) -
      // decoding happens below, separately, for the logical byte stream
      // everything else consumes.
      this.trace?.("RX", chunk);
      for (const raw of chunk) {
        this.trackWakeupCapability(raw);
        const b = this.unescapeByte(raw);
        if (b === undefined) {
          continue;
        }
        const waiter = this.waiters.shift();
        if (waiter) {
          waiter(b);
        } else {
          this.queue.push(b);
        }
      }
    });
  }

  // Watches the raw byte stream for WAKEUP_SIG followed by a capability
  // byte (v0.2.2 §2) and flips on escape decoding if the peer announces it.
  // Runs alongside the normal queue/waiter delivery below, not instead of
  // it - callers that do their own explicit WAKEUP_SIG matching (e.g.
  // handshakeAsReceiver) still see every byte exactly as before.
  private trackWakeupCapability(raw: number): void {
    if (this.awaitingCapabilityByte) {
      this.awaitingCapabilityByte = false;
      if (raw === WAKEUP_CAP_ESCAPED) {
        this.escapeDecodingEnabled = true;
      }
      return;
    }
    if (raw === WAKEUP_SIG[this.wakeupMatchLen]) {
      this.wakeupMatchLen += 1;
      if (this.wakeupMatchLen === WAKEUP_SIG.length) {
        this.wakeupMatchLen = 0;
        this.awaitingCapabilityByte = true;
      }
    } else {
      this.wakeupMatchLen = raw === WAKEUP_SIG[0] ? 1 : 0;
    }
  }

  // Decodes one raw wire byte per the v0.2.2 escaping scheme. Returns the
  // decoded byte, or undefined if this raw byte was purely an escape
  // introducer (state advanced, nothing to emit yet). A no-op passthrough
  // until escapeDecodingEnabled is set - see trackWakeupCapability.
  private unescapeByte(raw: number): number | undefined {
    if (!this.escapeDecodingEnabled) {
      return raw;
    }
    if (this.escapeState === "lit") {
      this.escapeState = "normal";
      return raw;
    }
    if (this.escapeState === "high") {
      this.escapeState = "normal";
      return raw | 0x80;
    }
    if (raw === ESC_LIT) {
      this.escapeState = "lit";
      return undefined;
    }
    if (raw === ESC_HIGH) {
      this.escapeState = "high";
      return undefined;
    }
    return raw;
  }

  baudRate(): number {
    return this.port.baudRate;
  }

  async close(): Promise<void> {
    await new Promise<void>((resolve) => {
      if (!this.port.isOpen) {
        resolve();
        return;
      }
      this.port.close(() => resolve());
    });
  }

  async writeBytes(bytes: Uint8Array): Promise<void> {
    this.trace?.("TX", Buffer.from(bytes));
    await new Promise<void>((resolve, reject) => {
      this.port.write(Buffer.from(bytes), (err) => {
        if (err) {
          reject(err);
          return;
        }
        this.port.drain((drainErr) => {
          if (drainErr) {
            reject(drainErr);
            return;
          }
          resolve();
        });
      });
    });
  }

  async clearInput(): Promise<void> {
    this.queue.length = 0;
    await new Promise<void>((resolve) => {
      this.port.flush(() => resolve());
    });
  }

  async readByteTimeout(timeoutMs: number): Promise<number | undefined> {
    if (this.queue.length > 0) {
      return this.queue.shift();
    }

    return new Promise<number | undefined>((resolve) => {
      const timer = setTimeout(() => {
        const idx = this.waiters.indexOf(done);
        if (idx >= 0) {
          this.waiters.splice(idx, 1);
        }
        resolve(undefined);
      }, timeoutMs);

      const done = (value: number | undefined) => {
        clearTimeout(timer);
        resolve(value);
      };

      this.waiters.push(done);
    });
  }

  async echoCanAndDrain(): Promise<void> {
    await this.writeBytes(Uint8Array.from([CTRL_CAN]));
    await sleep(50);
    await this.clearInput();
  }

  async sendControl(ctrl: number, seq?: number): Promise<void> {
    if (typeof seq === "number") {
      await this.writeBytes(Uint8Array.from([ctrl, seq & 0xff]));
      return;
    }
    await this.writeBytes(Uint8Array.from([ctrl]));
  }

  async recvControl(timeoutMs: number): Promise<Control> {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
      const remaining = Math.max(1, deadline - Date.now());
      const b = await this.readByteTimeout(Math.min(remaining, 2000));
      if (typeof b !== "number") {
        continue;
      }

      if (b === CTRL_ACK || b === CTRL_NAK) {
        const rem = Math.max(1, deadline - Date.now());
        const seq = await this.readByteTimeout(Math.min(rem, 2000));
        if (typeof seq !== "number") {
          throw new Error("Timeout waiting for sequence byte");
        }
        return b === CTRL_ACK ? { kind: "ack", seq } : { kind: "nak", seq };
      }

      if (b === CTRL_RDY) {
        return { kind: "rdy" };
      }
      if (b === CTRL_FIN) {
        return { kind: "fin" };
      }
      if (b === CTRL_CAN) {
        await this.echoCanAndDrain();
        return { kind: "can" };
      }
    }

    throw new Error("Timeout waiting for control byte");
  }

  async recvFrame(timeoutMs: number, debug: boolean): Promise<FrameResult> {
    const deadline = Date.now() + timeoutMs;

    if (debug) {
      process.stderr.write(`DEBUG recvFrame `);
    }

    while (Date.now() < deadline) {
      const remaining = Math.max(1, deadline - Date.now());
      const b = await this.readByteTimeout(Math.min(remaining, 2000));

      if (debug) {
        process.stderr.write(` 0x${(b ?? 0).toString(16).padStart(2, "0")}`);
        // process.stderr.write(`${String.fromCharCode(b ?? 0)}`);
      }

      if (typeof b !== "number") {
        continue;
      }

      if (b === SOF) {
        return this.recvFrameAfterSof(deadline, debug);
      }
      if (b === CTRL_FIN) {
        return { kind: "fin" };
      }
      if (b === CTRL_CAN) {
        await this.echoCanAndDrain();
        return { kind: "cancel" };
      }
    }

    throw new Error("Timeout waiting for SOF");
  }

  async recvFrameAfterSof(deadlineMs: number, debug: boolean): Promise<FrameResult> {

    if (debug) {
      process.stderr.write(`\nDEBUG recvFrameAfterSof `);
    }

    const seq = await this.readByteBefore(deadlineMs, "sequence");
    const lenH = await this.readByteBefore(deadlineMs, "len high");
    const lenL = await this.readByteBefore(deadlineMs, "len low");
    const length = ((lenH & 0xff) << 8) | (lenL & 0xff);

    const payload = await this.readExactBefore(length, deadlineMs);
    const crcH = await this.readByteBefore(deadlineMs, "crc high");
    const crcL = await this.readByteBefore(deadlineMs, "crc low");
    const rxCrc = ((crcH & 0xff) << 8) | (crcL & 0xff);

    const crcData = Buffer.alloc(3 + payload.length);
    crcData[0] = seq;
    crcData[1] = lenH;
    crcData[2] = lenL;
    payload.copy(crcData, 3);

    const calc = crc16Ccitt(crcData);
    if (debug) {
      process.stderr.write(`\nDEBUG recvFrameAfterSof seq=0x${seq.toString(16).padStart(2, "0")} len=${length} ` +
        `lenH=0x${lenH.toString(16).padStart(2, "0")} lenL=0x${lenL.toString(16).padStart(2, "0")} ` +
        `calcCrc=0x${calc.toString(16).padStart(4, "0")} rxCrc=0x${rxCrc.toString(16).padStart(4, "0")}\n`);
    }
    
    if (calc !== rxCrc) {
      throw new Error(
        `CRC mismatch: calc=0x${calc.toString(16).padStart(4, "0")} rx=0x${rxCrc
          .toString(16)
          .padStart(4, "0")} seq=0x${seq.toString(16).padStart(2, "0")} ` +
          `lenH=0x${lenH.toString(16).padStart(2, "0")} lenL=0x${lenL.toString(16).padStart(2, "0")} ` +
          `len=${length} payload=${payload.toString("hex")}`
      );
    }

    return { kind: "data", seq, payload };
  }

  private async readByteBefore(deadlineMs: number, label: string): Promise<number> {
    const rem = deadlineMs - Date.now();
    if (rem <= 0) {
      throw new Error(`Timeout waiting for ${label}`);
    }
    const b = await this.readByteTimeout(Math.min(rem, 2000));
    if (typeof b !== "number") {
      throw new Error(`Timeout waiting for ${label}`);
    }
    return b;
  }

  private async readExactBefore(length: number, deadlineMs: number): Promise<Buffer> {
    const out = Buffer.alloc(length);
    for (let i = 0; i < length; i += 1) {
      out[i] = await this.readByteBefore(deadlineMs, "payload");
    }
    return out;
  }
}
