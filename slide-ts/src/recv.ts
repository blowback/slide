import { promises as fs } from "node:fs";
import path from "node:path";
import {
  CTRL_ACK,
  CTRL_FIN,
  CTRL_NAK,
  CTRL_RDY,
  makeTraceLogger,
  openSerial,
  SerialSession,
  sleep,
  WAKEUP_SIG,
  WIN_SIZE
} from "./protocol.js";

type RecvResult =
  | { kind: "ok"; bytes: number }
  | { kind: "fin" }
  | { kind: "cancelled" }
  | { kind: "error" };

export async function recvSession(
  portName: string,
  files: string[],
  baud: number,
  outputDir: string,
  debug: boolean,
  generic_cpm: boolean,
  trace?: string
): Promise<void> {
  console.log("SLIDE v0.2 - Serial Line Inter-Device Exchange");
  console.log(`  Port:   ${portName} @ ${baud} baud`);
  console.log("  Mode:   receive");
  console.log(`  Output: ${outputDir}`);
  console.log("");

  await fs.mkdir(outputDir, { recursive: true });
  if (trace) {
    console.log(`  Trace:  ${trace}`);
  }
  const session = await openSerial(portName, baud, debug, trace ? makeTraceLogger(trace) : undefined);

  if (generic_cpm && files.length > 0) {
    const filelist = files.join(" ");
    console.log("Sending command to Z80... SLIDECPM S " + filelist);
    await session.writeBytes(new Uint8Array(`SLIDECPM S ${filelist}\n`.split("").map((c) => c.charCodeAt(0))));

  }

  try {
    await handshakeAsReceiver(session, debug);

    let fileCount = 0;
    let totalBytes = 0;
    const start = Date.now();

    while (true) {
      console.log(`\n-- Waiting for file ${fileCount + 1}... --`);
      const result = await recvOneFile(session, outputDir, debug);

      if (result.kind === "ok") {
        fileCount += 1;
        totalBytes += result.bytes;
        continue;
      }
      if (result.kind === "fin") {
        await session.sendControl(CTRL_FIN);
        if (debug) {
          console.error("DEBUG sent FIN echo");
        }
        break;
      }
      if (result.kind === "cancelled") {
        console.log("Cancelled by Z80.");
        break;
      }

      console.log("File transfer failed, aborting session.");
      break;
    }

    const elapsed = (Date.now() - start) / 1000;
    console.log("");
    console.log(`OK ${fileCount} file(s) received, ${totalBytes} bytes in ${elapsed.toFixed(1)}s`);
  } finally {
    // On a protocol error the Z80 side is often still mid-transmission (e.g.
    // a debug dump queued up behind the frame that just failed) - closing
    // the port immediately cuts the trace off mid-line. Give trailing bytes
    // a beat to arrive and get traced before tearing the port down; only
    // worth the wait when there's a trace to complete.
    if (trace) {
      await sleep(1000);
    }
    await session.close();
  }
}

async function handshakeAsReceiver(session: SerialSession, debug: boolean): Promise<void> {
  process.stdout.write("Waiting for Z80 sender (start SLIDE S <file> on Z80 now)... ");

  const deadline = Date.now() + 60000;

  let matchLen = 0;
  while (matchLen < WAKEUP_SIG.length) {
    if (Date.now() >= deadline) {
      throw new Error("Timeout waiting for Z80 wakeup signature");
    }
    const remaining = Math.max(1, deadline - Date.now());
    const b = await session.readByteTimeout(Math.min(remaining, 2000));
    if (typeof b !== "number") {
      continue;
    }
    if (b === WAKEUP_SIG[matchLen]) {
      matchLen += 1;
    } else {
      matchLen = b === WAKEUP_SIG[0] ? 1 : 0;
    }
  }
  if (debug) {
    console.error("DEBUG got wakeup signature");
  }

  while (Date.now() < deadline) {
    try {
      const ctrl = await session.recvControl(2000);
      if (ctrl.kind === "rdy") {
        // Echo RDY back before doing anything else - the Z80 may be
        // waiting on this with little to no retry budget (see the header-
        // ACK retry fix in slidecpm.asm), so minimize the delay. Only
        // after that's out the door do we pause briefly and drop whatever
        // arrived in the meantime, to start the actual transfer with a
        // clean queue.
        await session.sendControl(CTRL_RDY);
        console.log("connected");
        await sleep(50);
        await session.clearInput();
        return;
      }
      if (ctrl.kind === "can") {
        console.log("cancelled by Z80");
        throw new Error("Session cancelled by peer");
      }
      if (debug) {
        console.error(`DEBUG handshake got ${ctrl.kind}`);
      }
    } catch {
      // keep waiting
    }
  }

  throw new Error("Timeout waiting for Z80 ready signal");
}

async function recvOneFile(
  session: SerialSession,
  outputDir: string,
  debug: boolean
): Promise<RecvResult> {
  let filename: string;
  let filesize: number;

  try {
    const frame = await session.recvFrame(30000, debug);
    if (frame.kind === "fin") {
      return { kind: "fin" };
    }
    if (frame.kind === "cancel") {
      return { kind: "cancelled" };
    }

    if (debug) {
      console.error(`DEBUG header seq=${frame.seq} len=${frame.payload.length}`);
    }

    const parsed = parseHeader(frame.payload);
    filename = parsed.filename;
    filesize = parsed.filesize;
  } catch (err) {
    if (debug) {
      console.error(`DEBUG header receive failed: ${(err as Error).message}`);
    }
    return { kind: "error" };
  }

  await session.sendControl(CTRL_ACK, 0);

  const filepath = path.join(outputDir, filename);
  console.log(`  ${filename} - ${filesize} bytes`);

  let expectedSeq = 1;
  const receivedData: number[] = [];
  let retries = 0;
  const maxRetries = 15;

  while (true) {
    try {
      const frame = await session.recvFrame(10000, debug);

      if (frame.kind === "fin") {
        return { kind: "error" };
      }
      if (frame.kind === "cancel") {
        return { kind: "cancelled" };
      }

      retries = 0;

      if (frame.payload.length === 0) {
        await session.sendControl(CTRL_ACK, expectedSeq & 0xff);
        break;
      }

      if (frame.seq !== (expectedSeq & 0xff)) {
        if (debug) {
          console.error(`DEBUG seq mismatch got=${frame.seq} expected=${expectedSeq & 0xff}`);
        }
        await session.sendControl(CTRL_NAK, expectedSeq & 0xff);
        continue;
      }

      for (const b of frame.payload) {
        receivedData.push(b);
      }

      expectedSeq = (expectedSeq + 1) & 0xff;

      if (((expectedSeq - 1) & (WIN_SIZE - 1)) === 0) {
        await session.sendControl(CTRL_ACK, frame.seq);
      }

      if (debug) {
        const shown = Math.min(receivedData.length, filesize);
        console.error(`DEBUG frame seq=${frame.seq} len=${frame.payload.length} total=${shown}`);
      }
    } catch (err) {
      if (debug) {
        console.error(`DEBUG frame receive failed: ${(err as Error).message}`);
      }
      retries += 1;
      if (retries >= maxRetries) {
        return { kind: "error" };
      }
      await session.sendControl(CTRL_NAK, expectedSeq & 0xff);
    }
  }

  const out = Buffer.from(receivedData.slice(0, filesize));
  await fs.writeFile(filepath, out);
  console.log(`  saved ${filepath}`);

  return { kind: "ok", bytes: filesize };
}

function parseHeader(payload: Buffer): { filename: string; filesize: number } {
  const nullIdx = payload.indexOf(0);
  if (nullIdx < 0) {
    throw new Error("No null terminator in header");
  }
  if (payload.length < nullIdx + 5) {
    throw new Error("Header too short for file size");
  }

  const filename = payload.subarray(0, nullIdx).toString("utf8");
  const filesize = payload.readUInt32LE(nullIdx + 1);
  return { filename, filesize };
}
