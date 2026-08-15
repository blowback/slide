mod protocol;
mod send;
mod recv;
mod probe;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "slide", version, about = "SLIDE - Serial Line Inter-Device file Exchange")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Send file(s) to Z80/CP/M
    Send {
        /// Serial port (e.g., /dev/ttyUSB0, COM3)
        port: String,
        /// File(s) to send
        #[arg(required = true)]
        files: Vec<String>,
        /// Baud rate
        #[arg(long, default_value_t = 19200)]
        baud: u32,
        /// Show wire-level debug output
        #[arg(long)]
        debug: bool,
        /// Use generic CP/M protocol variant
        #[arg(long)]
        generic_cpm: bool,
    },
    /// Receive file(s) from Z80/CP/M
    Recv {
        /// Serial port (e.g., /dev/ttyUSB0, COM3)
        port: String,
        /// Files(s) to receive
        #[arg(required = false)]
        files: Vec<String>,
        /// Baud rate
        #[arg(long, default_value_t = 19200)]
        baud: u32,
        /// Directory for received files
        #[arg(long, default_value = ".")]
        output_dir: String,
        /// Show wire-level debug output
        #[arg(long)]
        debug: bool,
        /// Use generic CP/M protocol variant
        #[arg(long)]
        generic_cpm: bool,
    },
    /// Probe a peer for v0.3 command-channel support (wire v0.3 §2)
    Probe {
        /// Serial port (e.g., /dev/ttyUSB0, COM3)
        port: String,
        /// Baud rate
        #[arg(long, default_value_t = 19200)]
        baud: u32,
        /// Probe attempts before giving up
        #[arg(long, default_value_t = 3)]
        attempts: u32,
        /// Milliseconds to wait for each echo
        #[arg(long, default_value_t = 500)]
        timeout: u64,
        /// Milliseconds to wait after handshake before probing, to clear
        /// the post-RDY FIFO flush
        #[arg(long, default_value_t = 100)]
        settle: u64,
        /// Type this at the peer's CP/M prompt to start it, instead of
        /// requiring a separate terminal (e.g. "slide r" or "b:slide r")
        #[arg(long, value_name = "CMD")]
        start_cmd: Option<String>,
        /// Command to issue if the peer supports them
        #[arg(long, default_value = "nop",
              value_parser = ["nop", "vols", "dir", "del", "ren"])]
        cmd: String,
        /// Drive to list with --cmd dir: A-P or 0-15. Omit for the drive
        /// the MicroBeast currently has selected
        #[arg(long)]
        drive: Option<String>,
        /// User number to list with --cmd dir: 0-15. Omit for the current one
        #[arg(long)]
        user: Option<String>,
        /// Filename or pattern, e.g. "*.BAK". Filters --cmd dir; required
        /// for --cmd del and names the existing file for --cmd ren
        #[arg(long)]
        r#match: Option<String>,
        /// New name for --cmd ren
        #[arg(long)]
        to: Option<String>,
        /// Allow a wildcard --cmd del without confirmation
        #[arg(long)]
        yes: bool,
        /// After probing, send FILE to prove the session still works
        #[arg(long, value_name = "FILE")]
        then_send: Option<String>,
        /// Probe again after the file transfer, covering the v0.3 §2
        /// post-transfer position (use with --then-send)
        #[arg(long)]
        probe_after: bool,
        /// Leave the peer in its file loop instead of closing
        #[arg(long)]
        no_fin: bool,
        /// Show wire-level debug output
        #[arg(long)]
        debug: bool,
    },
}

fn main() {
    let cli = Cli::parse();
    let result = match cli.command {
        Commands::Send { port, files, baud, debug, generic_cpm } => {
            send::send_session(&port, &files, baud, debug, generic_cpm)
        }
        Commands::Recv { port, files, baud, output_dir, debug, generic_cpm } => {
            recv::recv_session(&port, &files, baud, &output_dir, debug, generic_cpm)
        }
        Commands::Probe { port, baud, attempts, timeout, settle, start_cmd, cmd, drive, user,
                          r#match, to, yes, then_send, probe_after, no_fin, debug } => {
            probe::probe_session(&port, baud, attempts, timeout, settle,
                                 start_cmd.as_deref(), &cmd,
                                 drive.as_deref(), user.as_deref(),
                                 r#match.as_deref(), to.as_deref(), yes,
                                 then_send.as_deref(), probe_after, no_fin, debug)
        }
    };
    if let Err(e) = result {
        eprintln!("\x1b[31merror:\x1b[0m {e:#}");
        std::process::exit(1);
    }
}
