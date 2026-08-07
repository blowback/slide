//! Command-channel capability probe (wire v0.3 §2).
//!
//! Sends CTRL_ENQ (0x05) to a peer sitting in receive mode and reports what
//! comes back. Its main job right now is to confirm the v0.3 §8
//! compatibility claim against real v0.2.1 firmware:
//!
//!   - a v0.2.1 peer skips the byte as stray and answers nothing, and
//!   - the session is undamaged afterwards, which we demonstrate by
//!     completing a normal FIN exchange (and optionally a whole file
//!     transfer) once the probe has come back empty.

use anyhow::{bail, Result};
use console::style;
use std::io::Write;
use std::path::Path;
use std::time::Duration;

use crate::protocol::*;
use crate::send::send_file;

fn hex(bytes: &[u8]) -> String {
    if bytes.is_empty() {
        "(none)".to_string()
    } else {
        bytes
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect::<Vec<_>>()
            .join(" ")
    }
}

fn outcome_name(outcome: &ProbeOutcome) -> &'static str {
    match outcome {
        ProbeOutcome::Supported { .. } => "supported",
        ProbeOutcome::Unsupported => "unsupported",
        ProbeOutcome::Malformed { .. } => "malformed",
        ProbeOutcome::Cancelled => "cancelled",
    }
}

/// Print the probe verdict and the raw evidence behind it.
fn report(result: &ProbeResult, label: &str) {
    println!();
    println!("{}", style(format!("--- Probe result ({label}) ---")).bold());
    println!("  Outcome:  {}", outcome_name(&result.outcome));
    println!("  Attempts: {}", result.attempts);
    println!("  Queued before probe: {}", hex(&result.discarded));
    println!("  Stray bytes during probe window: {}", hex(&result.stray));

    match &result.outcome {
        ProbeOutcome::Supported { major, minor, ver_raw } => {
            println!(
                "  VER raw:  {} ('{}')",
                hex(ver_raw),
                String::from_utf8_lossy(ver_raw)
            );
            println!("  Version:  major {major}, minor {minor}");
            println!("  We speak: major {CMDSET_MAJOR}, minor {CMDSET_MINOR}");

            if !result.usable() {
                println!();
                println!("  Peer's major version is not one we know. Per v0.3 §2, opcode");
                println!("  and record encodings may have changed, so commands MUST NOT");
                println!("  be sent. Falling back as if there had been no echo.");
            } else if *minor > CMDSET_MINOR {
                println!();
                println!("  Peer is a newer minor. The command set is additive within a");
                println!("  major, so every opcode we know still means what we think.");
            } else if *minor < CMDSET_MINOR {
                println!();
                println!("  Peer is an older minor. Restrict to opcodes defined at or");
                println!("  below minor {minor}.");
            }
        }
        ProbeOutcome::Unsupported => {
            println!();
            println!("  No echo. This is the expected v0.2.1 result: the peer skipped");
            println!("  0x05 as a stray byte and stayed in its file loop. Commands MUST");
            println!("  NOT be sent (v0.3 §6) — an unannounced frame would be parsed as");
            println!("  a file header and would put junk on the disk.");
        }
        ProbeOutcome::Malformed { ver_raw, reason } => {
            println!("  VER raw:  {}", hex(ver_raw));
            println!("  Detail:   {reason}");
            println!();
            println!("  ENQ was echoed but VER did not arrive as four ASCII digits.");
            println!("  v0.3 §2 says treat this as no command support rather than");
            println!("  guessing at the version.");
        }
        ProbeOutcome::Cancelled => {
            println!();
            println!("  Peer cancelled during the probe. CAN has been echoed and the");
            println!("  wire drained (v0.2.1 §2); both sides are back at idle.");
        }
    }
    println!();
}

/// Handshake, probe, optionally transfer a file, then close the session.
#[allow(clippy::too_many_arguments)]
pub fn probe_session(
    port_name: &str,
    baud: u32,
    attempts: u32,
    echo_timeout_ms: u64,
    settle_ms: u64,
    start_cmd: Option<&str>,
    then_send: Option<&str>,
    probe_after: bool,
    skip_fin: bool,
    debug: bool,
) -> Result<()> {
    println!(
        "{} — command-channel probe (wire v0.3 §2)",
        style("SLIDE").cyan().bold()
    );
    println!("  Port:  {} @ {} baud", style(port_name).yellow(), baud);
    println!(
        "  Probe: ENQ 0x{CTRL_ENQ:02X}, {attempts} attempt(s), {echo_timeout_ms}ms echo timeout"
    );
    println!();

    if let Some(f) = then_send {
        if !Path::new(f).exists() {
            bail!("File not found: {f}");
        }
    }

    let mut port = open_serial(port_name, baud)?;

    match start_cmd {
        Some(cmd) => {
            println!("Typing {} at the CP/M prompt...", style(cmd).yellow());
            send_start_command(port.as_mut(), cmd, debug)?;
        }
        None => println!("Waiting for Z80 (start SLIDE R on Z80 now)..."),
    }

    let hs = handshake_as_sender(port.as_mut(), Duration::from_secs(60), debug)?;
    if !hs.connected {
        bail!("No RDY echo from Z80 (handshake timeout or cancel)");
    }
    println!(
        "{} Z80 connected. ({} stray byte(s) skipped, wakeup signature {})",
        style("✓").green().bold(),
        hs.strays,
        if hs.wakeup_seen { "seen" } else { "not seen" }
    );

    // The probe goes exactly where a file header or FIN would go.
    let result = probe_command_support(
        port.as_mut(),
        attempts,
        Duration::from_millis(echo_timeout_ms),
        Duration::from_millis(settle_ms),
        debug,
    )?;
    report(&result, "after handshake");

    if result.outcome == ProbeOutcome::Cancelled {
        bail!("Peer cancelled during the probe");
    }

    // --- Prove the session survived the probe ---
    let mut session_ok = true;

    if let Some(filename) = then_send {
        println!(
            "{}",
            style(format!(
                "--- Sending {filename} to confirm the session still works ---"
            ))
            .bold()
        );
        match send_file(port.as_mut(), filename, debug) {
            Ok(_) => println!(),
            Err(e) => {
                println!("  File transfer FAILED after the probe: {e:#}");
                session_ok = false;
            }
        }
    }

    // v0.3 §2 lists three legal probe positions. The one above covers
    // "right after the RDY handshake"; this covers "after a completed file
    // transfer", where the residue risk is different — in receive mode the
    // preceding transfer's ACKs flow *from* the server, so anything the
    // client left unread lands in this probe's window.
    if probe_after {
        if then_send.is_none() {
            println!(
                "{}",
                style("Note: --probe-after without --then-send re-probes the same position.")
                    .yellow()
            );
        }
        let after = probe_command_support(
            port.as_mut(),
            attempts,
            Duration::from_millis(echo_timeout_ms),
            Duration::from_millis(settle_ms),
            debug,
        )?;
        report(&after, "after file transfer");

        if after.outcome == ProbeOutcome::Cancelled {
            bail!("Peer cancelled during the post-transfer probe");
        }
        if outcome_name(&after.outcome) != outcome_name(&result.outcome) {
            println!(
                "  {} post-transfer outcome differs from the post-handshake one.",
                style("NOTE:").yellow().bold()
            );
        }
        if !after.discarded.is_empty() {
            println!(
                "  {} {} byte(s) were still queued from the transfer.",
                style("NOTE:").yellow().bold(),
                after.discarded.len()
            );
        }
    }

    if skip_fin {
        println!("Skipping FIN exchange (--no-fin). The Z80 is still in its file loop.");
        return Ok(());
    }

    println!("{}", style("--- FIN exchange (session-intact check) ---").bold());
    port.write_all(&[CTRL_FIN])?;
    port.flush()?;

    match recv_control(port.as_mut(), Duration::from_secs(5)) {
        Ok(Control::Fin) => {
            println!("  FIN echoed. The peer was still in its file loop and exited");
            println!("  cleanly — the probe did not disturb the session.");
        }
        Ok(Control::Can) => {
            println!("  Peer cancelled instead of echoing FIN.");
            session_ok = false;
        }
        Ok(other) => {
            println!("  Unexpected reply to FIN: {other:?}");
            session_ok = false;
        }
        Err(_) => {
            println!("  No FIN echo within 5s. The probe may have disturbed the peer —");
            println!("  worth investigating before trusting the compatibility claim.");
            session_ok = false;
        }
    }

    println!();
    let name = outcome_name(&result.outcome);
    if session_ok {
        println!(
            "{} probe outcome '{name}', session intact.",
            style("VERDICT:").green().bold()
        );
        Ok(())
    } else {
        println!(
            "{} probe outcome '{name}', but the session did NOT close cleanly.",
            style("VERDICT:").red().bold()
        );
        bail!("session did not close cleanly after the probe");
    }
}
