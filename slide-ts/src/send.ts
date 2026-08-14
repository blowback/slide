import { promises as fs } from "node:fs";
import path from "node:path";
import {
  buildFrame,
  buildHeaderFrame,
  CTRL_CAN,
  CTRL_FIN,
  CTRL_RDY,
  FRAME_SIZE,
  makeTraceLogger,
  openSerial,
  SerialSession,
  TransferCancelledError,
  WIN_SIZE,
  sleep
} from "./protocol.js";

export async function sendSession(
  portName: string,
  files: string[],
  baud: number,
  debug: boolean,
  generic_cpm: boolean,
  trace?: string
): Promise<void> {
  console.log("SLIDE v0.2 - Serial Line Inter-Device Exchange");
  console.log(`  Port:  ${portName} @ ${baud} baud`);
  console.log("  Mode:  send");
  console.log(`  Files: ${files.length}`);
  console.log("");

  if (trace) {
    console.log(`  Trace: ${trace}`);
  }
  const session = await openSerial(portName, baud, debug, trace ? makeTraceLogger(trace) : undefined);
  try {
    await sendFiles(session, files, debug, generic_cpm);
  } finally {
    await session.close();
  }
}

/**
 * Core send logic, independent of how `session` was obtained - split out
 * from sendSession() so a caller that already holds an open SerialSession,
 * or wants progress/cancellation reporting, can drive a transfer directly.
 */
export async function sendFiles(
  session: SerialSession,
  files: string[],
  debug: boolean,
  generic_cpm: boolean,
  onProgress?: (bytesTransferred: number) => void,
  isCancelled?: () => boolean
): Promise<void> {
  for (const f of files) {
    await fs.access(f);
  }

  if (generic_cpm) {
    console.log("Sending SLIDECPM R command to Z80...");
    await session.writeBytes(new Uint8Array("SLIDECPM R\n".split("").map((c) => c.charCodeAt(0))));
  }

  console.log("Starting handshake...");

  await handshakeAsSender(session, debug);

  const start = Date.now();
  let totalBytes = 0;

  for (let i = 0; i < files.length; i += 1) {
    const filename = files[i];
    console.log(`\n-- File ${i + 1}/${files.length}: ${filename} --`);
    const bytesBeforeThisFile = totalBytes;
    totalBytes += await sendOneFile(
      session,
      filename,
      debug,
      (bytesInFile) => onProgress?.(bytesBeforeThisFile + bytesInFile),
      isCancelled
    );
  }

  if (debug) {
    console.error("DEBUG sending FIN");
  }
  await session.sendControl(CTRL_FIN);
  try {
    await session.recvControl(5000);
  } catch {
    // best effort
  }

  const elapsed = (Date.now() - start) / 1000;
  console.log("");
  console.log(`OK ${files.length} file(s) sent, ${totalBytes} bytes in ${elapsed.toFixed(1)}s`);
}

async function handshakeAsSender(session: SerialSession, debug: boolean): Promise<void> {
  process.stdout.write("Waiting for Z80 (start SLIDE R on Z80 now)... ");

  while (true) {
    await session.sendControl(CTRL_RDY);
    await sleep(1000);

    try {
      const ctrl = await session.recvControl(1000);
      if (ctrl.kind === "rdy") {
        break;
      }
      if (ctrl.kind === "can") {
        console.log("cancelled by Z80");
        throw new Error("Session cancelled by peer");
      }
      if (debug) {
        console.error(`DEBUG handshake got ${ctrl.kind}`);
      }
    } catch {
      // keep trying until receiver starts
    }
  }

  console.log("connected");
  await sleep(50);
  await session.clearInput();
}

async function sendOneFile(
  session: SerialSession,
  filename: string,
  debug: boolean,
  onProgress?: (bytesInFile: number) => void,
  isCancelled?: () => boolean
): Promise<number> {
  const fileStart = Date.now();
  const fileDataRaw = await fs.readFile(filename);
  const filesize = fileDataRaw.length;

  const padded = padForCpm(fileDataRaw);
  const frames: Array<{ seq: number; payload: Buffer }> = [];
  let seq = 1;
  for (let off = 0; off < padded.length; off += FRAME_SIZE) {
    const end = Math.min(off + FRAME_SIZE, padded.length);
    frames.push({ seq: seq & 0xff, payload: padded.subarray(off, end) });
    seq += 1;
  }

  const displayName = path.basename(filename);
  console.log(`  ${displayName} - ${filesize} bytes (${frames.length} frames)`);

  const header = buildHeaderFrame(filename, filesize);
  if (debug) {
    console.error(`DEBUG header (${header.length} bytes)`);
  }
  await session.writeBytes(header);

  const headerCtrl = await session.recvControl(60000);
  if (headerCtrl.kind === "can") {
    throw new Error("Z80 rejected file - check disk/drive");
  }
  if (headerCtrl.kind !== "ack") {
    throw new Error(`Expected ACK for header, got ${headerCtrl.kind}`);
  }

  await sleep(500);
  await session.clearInput();

  let sendIdx = 0;
  const eofSeq = frames.length > 0 ? ((frames[frames.length - 1].seq + 1) & 0xff) : 1;
  let eofSent = false;

  while (sendIdx < frames.length) {
    if (isCancelled?.()) {
      throw new TransferCancelledError(displayName);
    }
    const windowEnd = Math.min(sendIdx + WIN_SIZE, frames.length);

    for (let i = sendIdx; i < windowEnd; i += 1) {
      const frame = frames[i];
      await session.writeBytes(buildFrame(frame.seq, frame.payload));
      if (debug) {
        console.error(
          `DEBUG send frame seq=${frame.seq} len=${frame.payload.length} ${i + 1}/${frames.length}`
        );
      }
    }

    if (windowEnd >= frames.length && !eofSent) {
      await session.writeBytes(buildFrame(eofSeq, Buffer.alloc(0)));
      eofSent = true;
      if (debug) {
        console.error(`DEBUG send EOF frame seq=${eofSeq}`);
      }
    }

    let ctrl: Awaited<ReturnType<SerialSession["recvControl"]>>;
    try {
      while (true) {
        const c = await session.recvControl(10000);
        if (c.kind === "rdy") {
          if (debug) {
            console.error("DEBUG got RDY (flush/disk progress)");
          }
          continue;
        }
        ctrl = c;
        break;
      }
    } catch {
      ctrl = { kind: "rdy" };
    }

    if (debug) {
      if (ctrl.kind === "ack" || ctrl.kind === "nak") {
        console.error(`DEBUG got ${ctrl.kind} seq=${ctrl.seq}`);
      } else {
        console.error(`DEBUG got ${ctrl.kind}`);
      }
    }

    if (ctrl.kind === "can") {
      throw new Error("Z80 reported disk error - transfer aborted");
    }

    if (ctrl.kind === "ack") {
      while (sendIdx < frames.length) {
        const fseq = frames[sendIdx].seq;
        sendIdx += 1;
        if (fseq === ctrl.seq) {
          break;
        }
      }
      if (ctrl.seq === eofSeq) {
        sendIdx = frames.length;
      }
      onProgress?.(Math.min(sendIdx * FRAME_SIZE, filesize));
      continue;
    }

    if (ctrl.kind === "nak") {
      eofSent = false;
      const idx = frames.findIndex((f) => f.seq === ctrl.seq);
      sendIdx = idx >= 0 ? idx : sendIdx;
      continue;
    }

    // RDY or other control => retry current window
  }

  if (!eofSent) {
    await session.writeBytes(buildFrame(eofSeq, Buffer.alloc(0)));
  }
  onProgress?.(filesize);

  try {
    await session.recvControl(2000);
  } catch {
    // best effort
  }

  const elapsedSec = Math.max(0.001, (Date.now() - fileStart) / 1000);
  const throughput = Math.round(filesize / elapsedSec);
  const efficiency = Math.round((throughput / (session.baudRate() / 10)) * 100);
  if (debug) {
    console.error(`DEBUG ${throughput} B/s (~${efficiency}% efficiency)`);
  }

  return filesize;
}

function padForCpm(data: Buffer): Buffer {
  if (data.length % 128 === 0) {
    return data;
  }
  const pad = 128 - (data.length % 128);
  const out = Buffer.alloc(data.length + pad, 0x1a);
  data.copy(out, 0);
  return out;
}
