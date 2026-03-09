use anyhow::{bail, Context, Result};
use console::style;
use indicatif::{ProgressBar, ProgressStyle};
use std::fs;
use std::io::Write;
use std::path::Path;
use std::thread;
use std::time::{Duration, Instant};

use crate::protocol::*;

pub fn send_session(port_name: &str, files: &[String], baud: u32, debug: bool) -> Result<()> {
    println!(
        "{} v0.2 — Serial Line Inter-Device Exchange",
        style("SLIDE").cyan().bold()
    );
    println!("  Port:  {} @ {} baud", style(port_name).yellow(), baud);
    println!("  Mode:  {}", style("send").green().bold());
    println!("  Files: {}", files.len());
    println!();

    // Validate files exist
    for f in files {
        if !Path::new(f).exists() {
            bail!("File not found: {f}");
        }
    }

    let mut port = open_serial(port_name, baud)?;

    // Handshake: sender sends RDY, waits for receiver's echo
    let spin = ProgressBar::new_spinner();
    spin.set_style(
        ProgressStyle::default_spinner()
            .template("{spinner:.cyan} {msg}")
            .unwrap(),
    );
    spin.set_message("Waiting for Z80 (start SLIDE R on Z80 now)...");
    spin.enable_steady_tick(Duration::from_millis(100));

    loop {
        port.write_all(&[CTRL_RDY])?;
        port.flush()?;
        thread::sleep(Duration::from_secs(1));
        match recv_control(port.as_mut(), Duration::from_secs(1)) {
            Ok(Control::Rdy) => break,
            _ => continue,
        }
    }
    spin.finish_with_message(format!("{} Z80 connected.", style("✓").green().bold()));
    thread::sleep(Duration::from_millis(50));
    port.clear(serialport::ClearBuffer::Input)?;

    // Send each file
    let session_start = Instant::now();
    let mut total_bytes = 0u64;

    for (i, filename) in files.iter().enumerate() {
        println!(
            "\n{}",
            style(format!(
                "── File {}/{}: {} ──",
                i + 1,
                files.len(),
                filename
            ))
            .dim()
        );
        let bytes = send_file(port.as_mut(), filename, debug)?;
        total_bytes += bytes;
    }

    // FIN exchange
    if debug {
        eprintln!("  DEBUG sending FIN");
    }
    port.write_all(&[CTRL_FIN])?;
    port.flush()?;
    let _ = recv_control(port.as_mut(), Duration::from_secs(5));

    let elapsed = session_start.elapsed();
    println!();
    println!(
        "{} {} file(s) sent, {} bytes in {:.1}s",
        style("✓").green().bold(),
        files.len(),
        total_bytes,
        elapsed.as_secs_f64()
    );

    Ok(())
}

fn send_file(port: &mut dyn serialport::SerialPort, filename: &str, debug: bool) -> Result<u64> {
    let file_data_raw = fs::read(filename).context("Reading file")?;
    let filesize = file_data_raw.len() as u32;

    // Pad to 128-byte boundary for CP/M
    let mut file_data = file_data_raw;
    if file_data.len() % 128 != 0 {
        file_data.resize(file_data.len() + (128 - file_data.len() % 128), 0x1A);
    }

    // Split into frames
    let mut frames: Vec<(u8, Vec<u8>)> = Vec::new();
    let mut offset = 0;
    let mut seq: u16 = 1; // seq 0 is the header
    while offset < file_data.len() {
        let end = (offset + FRAME_SIZE).min(file_data.len());
        frames.push(((seq & 0xFF) as u8, file_data[offset..end].to_vec()));
        seq += 1;
        offset += FRAME_SIZE;
    }

    let total_frames = frames.len();
    let display_name = Path::new(filename)
        .file_name()
        .unwrap_or_default()
        .to_string_lossy();
    println!(
        "  {} — {} bytes ({} frames)",
        style(&*display_name).bold(),
        filesize,
        total_frames
    );

    // Send header
    let header = build_header_frame(filename, filesize);
    if debug {
        eprintln!("  DEBUG header ({} bytes): {:02x?}", header.len(), &header);
    }
    port.write_all(&header)?;
    port.flush()?;

    let ctrl = recv_control(port, Duration::from_secs(60))?;
    match ctrl {
        Control::Can => bail!("Z80 rejected file — check disk/drive"),
        Control::Ack(_) => {}
        other => bail!("Expected ACK for header, got {other:?}"),
    }

    // Give Z80 time to create the file
    thread::sleep(Duration::from_millis(500));
    port.clear(serialport::ClearBuffer::Input)?;

    // Progress bar
    let pb = ProgressBar::new(total_frames as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("  {bar:50.green/dim} {pos}/{len} frames  {msg}")
            .unwrap()
            .progress_chars("━╸─"),
    );

    // Stream data with sliding window
    let mut send_idx = 0;
    let eof_seq = if !frames.is_empty() {
        (frames.last().unwrap().0 as u16 + 1) as u8
    } else {
        1
    };
    let mut eof_sent = false;
    let start_time = Instant::now();

    while send_idx < total_frames {
        let window_end = (send_idx + WIN_SIZE).min(total_frames);

        for i in send_idx..window_end {
            let (seq_num, ref payload) = frames[i];
            let frame = build_frame(seq_num, payload);
            if debug {
                eprintln!(
                    "  DEBUG send frame seq={} len={} ({} bytes on wire)",
                    seq_num,
                    payload.len(),
                    frame.len()
                );
            }
            port.write_all(&frame)?;
        }

        // Append EOF to last window
        if window_end >= total_frames && !eof_sent {
            let eof_frame = build_frame(eof_seq, &[]);
            if debug {
                eprintln!("  DEBUG send EOF frame seq={eof_seq}");
            }
            port.write_all(&eof_frame)?;
            eof_sent = true;
        }
        port.flush()?;

        // Wait for response (skip RDY)
        let ctrl = loop {
            match recv_control(port, Duration::from_secs(10)) {
                Ok(Control::Rdy) => {
                    if debug {
                        eprintln!("  DEBUG got RDY (Z80 flushed to disk)");
                    }
                    continue;
                }
                Ok(c) => break c,
                Err(_) => {
                    pb.set_message(style("timeout, retrying...").yellow().to_string());
                    break Control::Rdy; // trigger retry
                }
            }
        };

        if debug {
            eprintln!("  DEBUG got {ctrl:?}");
        }

        match ctrl {
            Control::Can => {
                pb.abandon_with_message(style("disk error!").red().to_string());
                bail!("Z80 reported disk error — transfer aborted");
            }
            Control::Ack(acked_seq) => {
                // Advance send_idx past the acked sequence
                while send_idx < total_frames {
                    let fseq = frames[send_idx].0;
                    send_idx += 1;
                    if fseq == acked_seq {
                        break;
                    }
                }
                if acked_seq == eof_seq {
                    send_idx = total_frames;
                }
                pb.set_position(send_idx as u64);
            }
            Control::Nak(nak_seq) => {
                eof_sent = false;
                pb.set_message(
                    style(format!("NAK seq={nak_seq}, retransmitting..."))
                        .yellow()
                        .to_string(),
                );
                for (i, (fseq, _)) in frames.iter().enumerate() {
                    if *fseq == nak_seq {
                        send_idx = i;
                        break;
                    }
                }
            }
            _ => {} // RDY = retry the window
        }
    }

    // Send EOF if not already sent with last window
    if !eof_sent {
        let eof_frame = build_frame(eof_seq, &[]);
        port.write_all(&eof_frame)?;
        port.flush()?;
    }

    // Wait for final EOF ACK
    let _ = recv_control(port, Duration::from_secs(2));

    let elapsed = start_time.elapsed().as_secs_f64();
    let throughput = filesize as f64 / elapsed;
    let efficiency = throughput / (baud_to_bytes(port) as f64) * 100.0;

    pb.finish_with_message(format!(
        "{:.0} B/s ({:.0}% efficiency)",
        throughput, efficiency
    ));

    Ok(filesize as u64)
}

fn baud_to_bytes(port: &dyn serialport::SerialPort) -> u32 {
    port.baud_rate().unwrap_or(19200) / 10
}
