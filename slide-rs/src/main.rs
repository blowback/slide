mod protocol;
mod send;
mod recv;

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
    };
    if let Err(e) = result {
        eprintln!("\x1b[31merror:\x1b[0m {e:#}");
        std::process::exit(1);
    }
}
