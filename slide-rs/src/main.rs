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
    },
    /// Receive file(s) from Z80/CP/M
    Recv {
        /// Serial port (e.g., /dev/ttyUSB0, COM3)
        port: String,
        /// Baud rate
        #[arg(long, default_value_t = 19200)]
        baud: u32,
        /// Directory for received files
        #[arg(long, default_value = ".")]
        output_dir: String,
        /// Show wire-level debug output
        #[arg(long)]
        debug: bool,
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
        Commands::Send { port, files, baud, debug } => {
            send::send_session(&port, &files, baud, debug)
        }
        Commands::Recv { port, baud, output_dir, debug } => {
            recv::recv_session(&port, baud, &output_dir, debug)
        }
        Commands::Probe { port, baud, attempts, timeout, settle, start_cmd, then_send,
                          probe_after, no_fin, debug } => {
            probe::probe_session(&port, baud, attempts, timeout, settle,
                                 start_cmd.as_deref(), then_send.as_deref(),
                                 probe_after, no_fin, debug)
        }
    };
    if let Err(e) = result {
        eprintln!("\x1b[31merror:\x1b[0m {e:#}");
        std::process::exit(1);
    }
}
