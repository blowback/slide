import { recvSession } from "./recv.js";
import { sendSession } from "./send.js";

type SendCommand = {
  kind: "send";
  port: string;
  files: string[];
  baud: number;
  debug: boolean;
  generic_cpm: boolean;
  trace?: string;
};

type RecvCommand = {
  kind: "recv";
  port: string;
  files: string[];
  outputDir: string;
  baud: number;
  debug: boolean;
  generic_cpm: boolean;
  trace?: string;
};

type Command = SendCommand | RecvCommand;

async function main(): Promise<void> {
  const command = parseArgs(process.argv.slice(2));

  if (command.kind === "send") {
    await sendSession(
      command.port,
      command.files,
      command.baud,
      command.debug,
      command.generic_cpm,
      command.trace
    );
    return;
  }

  await recvSession(
    command.port,
    command.files,
    command.baud,
    command.outputDir,
    command.debug,
    command.generic_cpm,
    command.trace
  );
}

function parseArgs(args: string[]): Command {
  const [subcommand, ...rest] = args;

  if (!subcommand || subcommand === "-h" || subcommand === "--help") {
    printHelp();
    process.exit(0);
  }

  if (subcommand !== "send" && subcommand !== "recv") {
    throw new Error(`Unknown subcommand: ${subcommand}`);
  }

  let baud = 19200;
  let debug = false;
  let outputDir = ".";
  let generic_cpm = false;
  let trace: string | undefined;
  const positional: string[] = [];

  for (let i = 0; i < rest.length; i += 1) {
    const arg = rest[i];

    if (arg === "--baud") {
      const next = rest[i + 1];
      if (!next) {
        throw new Error("--baud requires a value");
      }
      baud = Number.parseInt(next, 10);
      if (!Number.isFinite(baud) || baud <= 0) {
        throw new Error(`Invalid baud value: ${next}`);
      }
      i += 1;
      continue;
    }

    if (arg === "--generic-cpm") {
      generic_cpm = true;
      continue;
    }

    if (arg === "--debug") {
      debug = true;
      continue;
    }

    if (arg === "--trace") {
      const next = rest[i + 1];
      if (!next) {
        throw new Error("--trace requires a file path");
      }
      trace = next;
      i += 1;
      continue;
    }

    if (arg === "--output-dir") {
      const next = rest[i + 1];
      if (!next) {
        throw new Error("--output-dir requires a value");
      }
      outputDir = next;
      i += 1;
      continue;
    }

    positional.push(arg);
  }

  if (subcommand === "send") {
    if (positional.length < 2) {
      throw new Error("Usage: slide-ts send <port> <file...> [--baud N] [--debug]");
    }
    const [port, ...files] = positional;
    return { kind: "send", port, files, baud, debug, generic_cpm, trace };
  }

  if (positional.length < 1) {
    throw new Error("Usage: slide-ts recv <port> [<file...>] [--output-dir dir] [--baud N] [--debug]");
  }

  return {
    kind: "recv",
    port: positional[0],
    files: positional.slice(1),
    outputDir,
    baud,
    debug,
    generic_cpm,
    trace
  };
}

function printHelp(): void {
  console.log("slide-ts - SLIDE TypeScript implementation");
  console.log("");
  console.log("Commands:");
  console.log("  slide-ts send <port> <file...> [--baud N] [--debug] [--generic-cpm] [--trace file]");
  console.log(
    "  slide-ts recv <port> [<file...>] [--output-dir dir] [--baud N] [--debug] [--generic-cpm] [--trace file]"
  );
  console.log("");
  console.log("Received files are optional to pull specific files from the Z80, ");
  console.log("otherwise only files will be pulled that are sent");
}

main().catch((err) => {
  const message = err instanceof Error ? err.message : String(err);
  console.error(`error: ${message}`);
  process.exit(1);
});
