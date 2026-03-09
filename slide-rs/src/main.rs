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
    };
    if let Err(e) = result {
        eprintln!("\x1b[31merror:\x1b[0m {e:#}");
        std::process::exit(1);
    }
}
