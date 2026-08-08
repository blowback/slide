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

/// Render a drive bitmap as CP/M drive letters.
fn drive_list(map: u16) -> String {
    let mut out: Vec<String> = Vec::new();
    for n in 0..16u32 {
        if map & (1 << n) != 0 {
            out.push(format!("{}:", (b'A' + n as u8) as char));
        }
    }
    if out.is_empty() {
        "(none)".to_string()
    } else {
        out.join(" ")
    }
}

/// Pretty-print a CMD_VOLS reply: one 5-byte record (v0.3 §5).
fn show_vols(records: &[u8]) {
    if records.len() < 5 {
        println!("  Malformed: expected a 5-byte record, got {}", records.len());
        println!("  Raw: {}", hex(records));
        return;
    }
    let present = u16::from(records[0]) | (u16::from(records[1]) << 8);
    let logged = u16::from(records[2]) | (u16::from(records[3]) << 8);
    println!("  Present: {}", drive_list(present));
    println!("  Logged:  {}", drive_list(logged));
    println!(
        "  Current: {}:",
        (b'A' + records[4].min(15)) as char
    );
}

/// Pretty-print a CMD_DIR reply: 16-byte records (v0.3 §5).
fn show_dir(records: &[u8]) {
    if records.len() % 16 != 0 {
        println!(
            "  Malformed: {} bytes is not a whole number of 16-byte records",
            records.len()
        );
        println!("  Raw: {}", hex(records));
        return;
    }
    if records.is_empty() {
        println!("  (empty directory)");
        return;
    }
    println!("  {:<4} {:<12} {:<5} {:>10}", "USER", "NAME", "ATTR", "BYTES");
    for r in records.chunks(16) {
        let name = String::from_utf8_lossy(&r[1..9]);
        let ext = String::from_utf8_lossy(&r[9..12]);
        let mut attr = String::new();
        if r[12] & 0x01 != 0 {
            attr.push_str("R/O ");
        }
        if r[12] & 0x02 != 0 {
            attr.push_str("SYS ");
        }
        if r[12] & 0x04 != 0 {
            attr.push_str("ARC ");
        }
        // Size is in 128-byte records — an upper bound in bytes, as for the
        // file-transfer header's size field.
        let recs = u32::from(r[13]) | (u32::from(r[14]) << 8) | (u32::from(r[15]) << 16);
        println!(
            "  {:<4} {:<12} {:<5} {:>10}",
            r[0],
            format!("{}.{}", name.trim_end(), ext.trim_end()),
            attr.trim_end(),
            recs * 128
        );
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


/// Turn a CP/M-ish glob into the 11-byte FCB form the wire uses: 8 name
/// bytes then 3 type bytes, space padded, `?` as the single-character
/// wildcard. `*` fills the rest of its field with `?`, as CP/M does.
fn parse_pattern(spec: &str) -> Result<Vec<u8>> {
    let up = spec.trim().to_ascii_uppercase();
    if up.is_empty() {
        bail!("empty pattern");
    }
    let (name, ext) = match up.split_once('.') {
        Some((n, e)) => (n.to_string(), e.to_string()),
        None => (up.clone(), String::new()),
    };
    fn field(src: &str, width: usize) -> Result<Vec<u8>> {
        let mut out: Vec<u8> = Vec::with_capacity(width);
        for ch in src.chars() {
            if out.len() == width {
                break;
            }
            if ch == '*' {
                while out.len() < width {
                    out.push(b'?');
                }
                break;
            }
            if !ch.is_ascii() || ch.is_ascii_control() || ch == ' ' {
                bail!("{ch:?} is not usable in a CP/M filename");
            }
            out.push(ch as u8);
        }
        while out.len() < width {
            out.push(b' ');
        }
        Ok(out)
    }
    let mut p = field(&name, 8)?;
    p.extend(field(&ext, 3)?);
    Ok(p)
}

fn has_wildcard(pattern: &[u8]) -> bool {
    pattern.contains(&b'?')
}

/// Render an 11-byte FCB pattern back as NAME.EXT for display.
fn show_pattern(p: &[u8]) -> String {
    let name = String::from_utf8_lossy(&p[0..8]).trim_end().to_string();
    let ext = String::from_utf8_lossy(&p[8..11]).trim_end().to_string();
    if ext.is_empty() { name } else { format!("{name}.{ext}") }
}

/// Parse a CP/M drive from the command line: "a", "A:", or a bare 0-15.
/// Returns 0..15, or 0xFF for "whatever the Beast currently has selected"
/// — the value v0.3 §5 reserves for that.
fn parse_drive(arg: Option<&str>) -> Result<u8> {
    let Some(raw) = arg else { return Ok(0xFF) };
    let t = raw.trim().trim_end_matches(':');
    if t.is_empty() {
        bail!("empty drive: {raw:?}");
    }
    let first = t.chars().next().unwrap();
    if t.chars().count() == 1 && first.is_ascii_alphabetic() {
        let n = first.to_ascii_uppercase() as u8 - b'A';
        if n > 15 {
            bail!("drive out of range: {raw:?} (A-P)");
        }
        return Ok(n);
    }
    match t.parse::<u8>() {
        Ok(n) if n <= 15 => Ok(n),
        Ok(_) => bail!("drive out of range: {raw:?} (0-15)"),
        Err(_) => bail!("cannot read {raw:?} as a drive — use A-P or 0-15"),
    }
}

/// Human-readable name for a drive operand.
fn drive_label(d: u8) -> String {
    if d == 0xFF {
        "current drive".to_string()
    } else {
        format!("{}:", (b'A' + d) as char)
    }
}

/// v0.3 §2: an echo obliges the client to send exactly one command frame.
/// Even when we only wanted to probe, the server is in command mode and
/// would read a file header as a command — so complete the exchange with
/// CMD_NOP. Returns false if the command itself failed.
fn honour_enq_obligation(
    port: &mut dyn serialport::SerialPort,
    result: &ProbeResult,
    cmd: &str,
    drive: u8,
    user: u8,
    pattern: Option<&Vec<u8>>,
    rename_to: Option<&Vec<u8>>,
    debug: bool,
) -> Result<bool> {
    if !result.usable() {
        return Ok(true);
    }
    // §5 operands always start [drive][user], 0xFF for "whatever is current"
    let (opcode, operands, label) = match cmd {
        "vols" => (CMD_VOLS, vec![], "CMD_VOLS".to_string()),
        "dir" => {
            let mut ops = vec![drive, user];
            let mut lbl = format!("CMD_DIR ({})", drive_label(drive));
            if let Some(p) = pattern {
                ops.extend_from_slice(p);
                lbl = format!("CMD_DIR ({} {})", drive_label(drive), show_pattern(p));
            }
            (CMD_DIR, ops, lbl)
        }
        "del" => {
            let p = pattern.expect("--match is required for del");
            let mut ops = vec![drive, user];
            ops.extend_from_slice(p);
            (CMD_DEL, ops,
             format!("CMD_DEL ({} {})", drive_label(drive), show_pattern(p)))
        }
        "ren" => {
            let from = pattern.expect("--match is required for ren");
            let to = rename_to.expect("--to is required for ren");
            let mut ops = vec![drive, user];
            ops.extend_from_slice(from);
            ops.extend_from_slice(to);
            (CMD_REN, ops,
             format!("CMD_REN ({} {} -> {})", drive_label(drive),
                     show_pattern(from), show_pattern(to)))
        }
        _ => (CMD_NOP, vec![], "CMD_NOP".to_string()),
    };
    println!("{}", style(format!("--- {label} ---")).bold());
    match send_command(port, opcode, &operands, debug) {
        Ok(reply) => {
            println!("  Status: 0x{:02X} — {}", reply.status, status_name(reply.status));
            if reply.status == 0 {
                match opcode {
                    CMD_VOLS => show_vols(&reply.records),
                    CMD_DIR => show_dir(&reply.records),
                    _ => println!("  (no records — probe-only exchange completed)"),
                }
            } else if !reply.records.is_empty() {
                println!("  Unexpected records with a non-OK status: {}", hex(&reply.records));
            }
            println!();
            Ok(true)
        }
        Err(e) => {
            println!("  Command FAILED: {e:#}");
            println!();
            Ok(false)
        }
    }
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
    cmd: &str,
    drive: Option<&str>,
    user: Option<&str>,
    r#match: Option<&str>,
    rename_to: Option<&str>,
    assume_yes: bool,
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

    let drive_n = parse_drive(drive)?;
    let user_n = parse_drive(user)?;   // same 0-15 / 0xFF shape
    let pattern = match r#match {
        Some(m) => Some(parse_pattern(m)?),
        None => None,
    };
    let to_pattern = match rename_to {
        Some(m) => Some(parse_pattern(m)?),
        None => None,
    };

    match cmd {
        "del" => {
            let Some(p) = pattern.as_ref() else {
                bail!("--cmd del needs --match; there is no delete-everything default");
            };
            // A wildcard delete on CP/M is unrecoverable, and --drive makes it
            // easy to aim at the wrong disk. Make the user say so out loud.
            if has_wildcard(p) && !assume_yes {
                bail!("--match {} is a wildcard and would delete every match on {}. \
Re-run with --yes if that is what you want.",
                      show_pattern(p), drive_label(drive_n));
            }
        }
        "ren" => {
            if pattern.is_none() {
                bail!("--cmd ren needs --match for the existing name");
            }
            if to_pattern.is_none() {
                bail!("--cmd ren needs --to for the new name");
            }
        }
        _ => {}
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

    // v0.3 §2: an echo obliges us to send exactly one command frame.
    let mut session_ok = true;
    if !honour_enq_obligation(port.as_mut(), &result, cmd, drive_n, user_n,
                              pattern.as_ref(), to_pattern.as_ref(), debug)? {
        session_ok = false;
    }

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
        if !honour_enq_obligation(port.as_mut(), &after, "nop", 0xFF, 0xFF,
                                  None, None, debug)? {
            session_ok = false;
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

    // 10s, not 5: the receiver closes the file (a directory write) after
    // acknowledging EOF and before returning to its file loop.
    match recv_control(port.as_mut(), Duration::from_secs(10)) {
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
            println!("  No FIN echo within 10s. The probe may have disturbed the peer —");
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drive_accepts_letters_and_numbers() {
        assert_eq!(parse_drive(Some("a")).unwrap(), 0);
        assert_eq!(parse_drive(Some("A")).unwrap(), 0);
        assert_eq!(parse_drive(Some("b:")).unwrap(), 1);
        assert_eq!(parse_drive(Some("B:")).unwrap(), 1);
        assert_eq!(parse_drive(Some("P")).unwrap(), 15);
        assert_eq!(parse_drive(Some("0")).unwrap(), 0);
        assert_eq!(parse_drive(Some("15")).unwrap(), 15);
        assert_eq!(parse_drive(Some(" b: ")).unwrap(), 1);
    }

    #[test]
    fn absent_drive_means_current() {
        // v0.3 §5 reserves 0xFF for "whatever the peer has selected".
        assert_eq!(parse_drive(None).unwrap(), 0xFF);
    }

    #[test]
    fn drive_rejects_out_of_range_and_junk() {
        assert!(parse_drive(Some("q")).is_err());   // past P
        assert!(parse_drive(Some("16")).is_err());
        assert!(parse_drive(Some("255")).is_err()); // must not sneak in as 0xFF
        assert!(parse_drive(Some("")).is_err());
        assert!(parse_drive(Some(":")).is_err());
        assert!(parse_drive(Some("ab")).is_err());
    }

    #[test]
    fn drive_label_reads_back() {
        assert_eq!(drive_label(0), "A:");
        assert_eq!(drive_label(1), "B:");
        assert_eq!(drive_label(15), "P:");
        assert_eq!(drive_label(0xFF), "current drive");
    }
}

#[cfg(test)]
mod pattern_tests {
    use super::*;

    fn p(s: &str) -> String {
        String::from_utf8(parse_pattern(s).unwrap()).unwrap()
    }

    #[test]
    fn patterns_become_fcb_fields() {
        assert_eq!(p("FOO.TXT"), "FOO     TXT");
        assert_eq!(p("foo.txt"), "FOO     TXT");   // upcased
        assert_eq!(p("FOO"), "FOO        ");       // no type
        assert_eq!(p("*.BAK"), "????????BAK");
        assert_eq!(p("*.*"), "???????????");
        assert_eq!(p("A?.COM"), "A?      COM");
        assert_eq!(p("SLIDE.*"), "SLIDE   ???");
        // over-long fields truncate, as CP/M does
        assert_eq!(p("VERYLONGNAME.EXTRA"), "VERYLONGEXT");
    }

    #[test]
    fn patterns_reject_junk() {
        assert!(parse_pattern("").is_err());
        assert!(parse_pattern("   ").is_err());
        assert!(parse_pattern("A B.TXT").is_err());  // embedded space
    }

    #[test]
    fn wildcards_are_detected() {
        // This gates the destructive-delete confirmation, so it must not
        // miss a wildcard that only appears in the type field.
        assert!(has_wildcard(&parse_pattern("*.BAK").unwrap()));
        assert!(has_wildcard(&parse_pattern("SLIDE.*").unwrap()));
        assert!(has_wildcard(&parse_pattern("A?.COM").unwrap()));
        assert!(!has_wildcard(&parse_pattern("FOO.TXT").unwrap()));
        assert!(!has_wildcard(&parse_pattern("FOO").unwrap()));
    }

    #[test]
    fn patterns_round_trip_for_display() {
        assert_eq!(show_pattern(&parse_pattern("FOO.TXT").unwrap()), "FOO.TXT");
        assert_eq!(show_pattern(&parse_pattern("FOO").unwrap()), "FOO");
        assert_eq!(show_pattern(&parse_pattern("*.BAK").unwrap()), "????????.BAK");
    }
}
