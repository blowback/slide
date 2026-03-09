use anyhow::{bail, Result};
use console::style;
use indicatif::{ProgressBar, ProgressStyle};
use std::fs;
use std::io::Write;
use std::path::Path;
use std::thread;
use std::time::{Duration, Instant};

use crate::protocol::*;

pub fn recv_session(port_name: &str, baud: u32, output_dir: &str, debug: bool) -> Result<()> {
    println!(
        "{} v0.2 — Serial Line Inter-Device Exchange",
        style("SLIDE").cyan().bold()
    );
    println!("  Port:   {} @ {} baud", style(port_name).yellow(), baud);
    println!("  Mode:   {}", style("receive").green().bold());
    println!("  Output: {}", output_dir);
    println!();

    fs::create_dir_all(output_dir)?;
    let mut port = open_serial(port_name, baud)?;

    // Handshake: wait for sender's RDY, echo back
    let spin = ProgressBar::new_spinner();
    spin.set_style(
        ProgressStyle::default_spinner()
            .template("{spinner:.cyan} {msg}")
            .unwrap(),
    );
    spin.set_message("Waiting for Z80 sender (start SLIDE S <file> on Z80 now)...");
    spin.enable_steady_tick(Duration::from_millis(100));

    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            spin.abandon_with_message(style("Timeout waiting for Z80").red().to_string());
            bail!("Timeout waiting for Z80 ready signal");
        }
        match recv_control(port.as_mut(), remaining.min(Duration::from_secs(2))) {
            Ok(Control::Rdy) => break,
            _ => continue,
        }
    }

    // Echo RDY back
    port.write_all(&[CTRL_RDY])?;
    port.flush()?;
    spin.finish_with_message(format!("{} Z80 connected.", style("✓").green().bold()));
    thread::sleep(Duration::from_millis(50));
    port.clear(serialport::ClearBuffer::Input)?;

    // Receive files until FIN
    let mut file_count = 0u32;
    let session_start = Instant::now();
    let mut total_bytes = 0u64;

    loop {
        println!(
            "\n{}",
            style(format!("── Waiting for file {}... ──", file_count + 1)).dim()
        );
        match recv_one_file(port.as_mut(), output_dir, debug)? {
            RecvResult::Ok(bytes) => {
                file_count += 1;
                total_bytes += bytes;
            }
            RecvResult::Fin => {
                send_control(port.as_mut(), CTRL_FIN, None)?;
                if debug {
                    eprintln!("  DEBUG sent FIN echo");
                }
                break;
            }
            RecvResult::Error => {
                println!("  File transfer failed, aborting session.");
                break;
            }
        }
    }

    let elapsed = session_start.elapsed();
    println!();
    println!(
        "{} {} file(s) received, {} bytes in {:.1}s",
        style("✓").green().bold(),
        file_count,
        total_bytes,
        elapsed.as_secs_f64()
    );

    Ok(())
}

enum RecvResult {
    Ok(u64),
    Fin,
    Error,
}

fn recv_one_file(port: &mut dyn serialport::SerialPort, output_dir: &str, debug: bool) -> Result<RecvResult> {
    // Receive header frame (seq 0)
    let (filename, filesize) = match recv_frame(port, Duration::from_secs(30))? {
        FrameResult::Fin => return Ok(RecvResult::Fin),
        FrameResult::Data(frame) => {
            if debug {
                eprintln!(
                    "  DEBUG header seq={} len={}: {:02x?}",
                    frame.seq,
                    frame.payload.len(),
                    &frame.payload
                );
            }
            parse_header(&frame.payload)?
        }
    };

    let filepath = Path::new(output_dir).join(&filename);
    println!(
        "  {} — {} bytes",
        style(&filename).bold(),
        filesize
    );

    // ACK header
    send_control(port, CTRL_ACK, Some(0))?;

    // Progress bar
    let pb = ProgressBar::new(filesize as u64);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("  {bar:50.green/dim} {bytes}/{total_bytes}  {msg}")
            .unwrap()
            .progress_chars("━╸─"),
    );

    // Receive data frames
    let mut expected_seq: u8 = 1;
    let mut received_data: Vec<u8> = Vec::with_capacity(filesize as usize);
    let mut retry_count = 0u32;
    let max_retries = 15;

    loop {
        let frame_result = recv_frame(port, Duration::from_secs(10));

        match frame_result {
            Err(e) => {
                retry_count += 1;
                if retry_count >= max_retries {
                    pb.abandon_with_message(style("too many retries").red().to_string());
                    return Ok(RecvResult::Error);
                }
                if debug {
                    eprintln!("  DEBUG error: {e}, NAK seq={expected_seq}");
                }
                pb.set_message(
                    style(format!("retry {retry_count}/{max_retries}"))
                        .yellow()
                        .to_string(),
                );
                send_control(port, CTRL_NAK, Some(expected_seq))?;
                continue;
            }
            Ok(FrameResult::Fin) => {
                pb.abandon_with_message(style("unexpected FIN").red().to_string());
                return Ok(RecvResult::Error);
            }
            Ok(FrameResult::Data(frame)) => {
                retry_count = 0;

                // EOF = zero-length payload
                if frame.payload.is_empty() {
                    if debug {
                        eprintln!("  DEBUG EOF frame seq={}", frame.seq);
                    }
                    send_control(port, CTRL_ACK, Some(expected_seq))?;
                    break;
                }

                // Sequence check
                if frame.seq != expected_seq {
                    if debug {
                        eprintln!(
                            "  DEBUG seq mismatch: got {} expected {}",
                            frame.seq, expected_seq
                        );
                    }
                    send_control(port, CTRL_NAK, Some(expected_seq))?;
                    continue;
                }

                received_data.extend_from_slice(&frame.payload);
                expected_seq = expected_seq.wrapping_add(1);

                if debug {
                    eprintln!(
                        "  DEBUG frame seq={} len={} total={}",
                        frame.seq,
                        frame.payload.len(),
                        received_data.len()
                    );
                }

                // ACK every WIN_SIZE frames
                if (expected_seq.wrapping_sub(1)) & (WIN_SIZE as u8 - 1) == 0 {
                    send_control(port, CTRL_ACK, Some(frame.seq))?;
                    if debug {
                        eprintln!("  DEBUG ACK seq={}", frame.seq);
                    }
                }

                pb.set_position(received_data.len().min(filesize as usize) as u64);
            }
        }
    }

    // Truncate to actual size (remove CP/M padding)
    let file_data = if received_data.len() > filesize as usize {
        &received_data[..filesize as usize]
    } else {
        &received_data
    };

    fs::write(&filepath, file_data)?;

    pb.finish_with_message(format!(
        "saved {}",
        style(filepath.display()).dim()
    ));

    Ok(RecvResult::Ok(filesize as u64))
}

fn parse_header(payload: &[u8]) -> Result<(String, u32)> {
    let null_idx = payload
        .iter()
        .position(|&b| b == 0)
        .ok_or_else(|| anyhow::anyhow!("No null terminator in header"))?;
    let filename = String::from_utf8(payload[..null_idx].to_vec())?;
    if payload.len() < null_idx + 5 {
        bail!("Header too short for file size");
    }
    let filesize = u32::from_le_bytes(payload[null_idx + 1..null_idx + 5].try_into()?);
    Ok((filename, filesize))
}
