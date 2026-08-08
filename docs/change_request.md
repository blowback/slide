---
  slide.com v0.5 — status messages invisible to remote serial users

  **Status: IMPLEMENTED — c397b07 (2026-05-13)**, "bugfix; console messages weren't
  going out the UART so you couldn't see them in BeasTTY". `print_user_msg` landed and
  the C_WRITESTR call sites were converted. Kept for the reasoning; not a live proposal.

  Problem

  When slide.com runs in receive mode driven by a remote PC over the serial port (e.g. Beastty, or any other PC-side SLIDE sender), the user never sees the per-file msg_done
  ("Transfer complete!") or the per-session msg_done_session ("Session complete.") messages. Same for msg_cancelled, msg_err_*, etc. Users running slide.com from a local CP/M
  console see them fine.

  Evidence

  Wire capture of a single-file PC → Z80 send (PC is the SLIDE sender, Z80 runs B:SLIDE r):

  PC → Z80:  CTRL_RDY                   (handshake)
  Z80 → PC:  CTRL_RDY                   (handshake echo)
  PC → Z80:  SOF + header frame
  Z80 → PC:  CTRL_ACK 0                 (header ack)
  PC → Z80:  data frame seq=1 + EOF seq=2
  Z80 → PC:  CTRL_ACK 2                 (EOF ack)
  PC → Z80:  CTRL_FIN
  Z80 → PC:  CTRL_FIN                   (echo)
  Z80 → PC:  \r\n B>                    (CCP prompt — slide.com has exited)

  Expected between CTRL_ACK 2 and the CCP prompt: ~50 bytes of \r\n Transfer complete! \r\n (msg_done) and \r\n Session complete. \r\n (msg_done_session). Observed: 0 bytes.

  uart_tx (slide.asm protocol output) reaches the PC correctly. BDOS C_WRITESTR (slide.asm status output) does not.

  Root cause

  slide.asm lines 83-91 (entry point):

  entry
          ; save/set IOBYTE to keep BIOS off the UART
          LD    HL, IOBYTE
          LD    A, (HL)
          LD    DE, iobyte_saved
          LD    (DE), A

          LD    A, 0b01010110   ; CON=BAT, RDR=PTR, LST=CRT
          LD    (HL), A

  slide.com rewrites the CP/M IOBYTE so CON=BAT (BIOS console output now routes via LST). LST is set to CRT. On MicroBeast configurations where the user's "console" is the serial
   UART (no local CRT mapped to the same port), every subsequent BDOS C_WRITESTR call writes into a black hole until the IOBYTE is restored at .out (line 119).

  That includes every status message slide.com emits during the session:

  - msg_done at .eof_ack (line 1092)
  - msg_done_session at .got_fin (line 933) and at the send-side equivalent (line 1266)
  - msg_done at the send-side .tx_done (line 1455)
  - msg_cancelled, msg_err_hdr, msg_err_file, msg_err_handshake, msg_err_abort

  For local-console users whose LST=CRT physically resolves to a visible device, this is invisible. For PC-driven users, all eight messages are silently dropped.

  Proposed fix

  Bracket each user-facing status print with an IOBYTE save/restore. Easiest as a helper:

  ; ============================================================================
  ; print_user_msg — emit a $-terminated string via the user's ORIGINAL console,
  ; bypassing slide.com's IOBYTE redirect so it reaches the same device the user
  ; is interacting with (local CRT for local users, serial UART for PC-driven).
  ; DE = pointer to $-terminated string. Trashes A, HL.
  ; ============================================================================
  print_user_msg
          ; switch IOBYTE back to the user's original setting
          LD    HL, IOBYTE
          LD    A, (HL)
          PUSH  AF                  ; save current (redirect) value
          LD    A, (iobyte_saved)
          LD    (HL), A

          LD    C, C_WRITESTR
          CALL  BDOS

          ; restore SLIDE's redirect IOBYTE
          POP   AF
          LD    HL, IOBYTE
          LD    (HL), A
          RET

  Then each call site changes from:

  LD    DE, msg_done
  LD    C, C_WRITESTR
  CALL  BDOS

  to:

  LD    DE, msg_done
  CALL  print_user_msg

  Eight call sites in total (search C_WRITESTR in slide.asm).

  Why this preserves the original design intent

  The IOBYTE redirect at entry was presumably defensive — to stop BIOS from accidentally interleaving console output with SLIDE protocol bytes on the same UART during a transfer.
   That guarantee is preserved: this fix only widens the route for the explicit user-status calls, all of which happen at protocol-quiescent moments (handshake, EOF ACK, FIN
  echo). The data-frame transmission path is unchanged.

  Compatibility

  - Local console users: unchanged behaviour — messages print to their normal console via the restored original IOBYTE.
  - PC-driven users (Beastty, slide-rs, slide-py, etc.): messages now reach the serial UART as expected; PC-side terminals display them inline with the CP/M prompt.
  - Wire protocol: unaffected. Status messages are after-the-fact human-readable text; no SLIDE consumer parses them.

  Test plan

  1. Local console: run B:SLIDE r from CP/M directly; confirm "Transfer complete!" and "Session complete." still appear on the local console.
  2. Remote PC: drive a single-file send from a PC SLIDE sender; confirm both messages now appear on the remote terminal between the FIN exchange and the next B> prompt.
  3. Error paths: trigger a CRC retry exhaustion (msg_err_abort), a peer-initiated CAN (msg_cancelled), and a handshake timeout (msg_err_handshake); confirm each prints on both
  local and remote.

  ---
