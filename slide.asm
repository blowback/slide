; ============================================================================
; SLIDE v0.6.0-dev - Serial Line Inter-Device (file) Exchange
; Custom file transfer protocol for Z80 / CP/M (wire protocol v0.3)
;
; v0.6.0-dev: wire v0.3 command channel (see docs/SPEC-v0.3.md). CTRL_ENQ
; is answered in the receive-mode file loop; CMD_NOP / CMD_VOLS / CMD_DIR
; are served from real BDOS/BIOS enumeration.
; Target: 8MHz Z80, TL16C550 UART with 16-byte FIFO, auto RTS/CTS flow control
;
; Usage:  SLIDE              — receive mode (default)
;         SLIDE R            — receive mode (explicit)
;         SLIDE S FILE.COM   — send FILE.COM to PC
;         SLIDE S A.COM B.DAT — send multiple files
;
; v0.5.2: zero-pad the final partial record on receive, so the unused tail
;   of the last 128-byte sector is deterministic instead of leaking stale
;   RXBUF data to disk. Round-tripped files now match the source byte-for-byte.
;
; v0.2.1 amendments (vs v0.2):
;   §1 Wakeup signature: emit ESC ^ S L I D E on entering SLIDE mode,
;      before any RDY/control/payload. Lets the host (e.g. Beastty)
;      detect that SLIDE.COM is alive without polling.
;   §2 Bidirectional CTRL_CAN: either side may initiate cancel; the
;      other side echoes CTRL_CAN back within ~500 ms. Idempotent —
;      a second CAN from the same side is a no-op.
; ============================================================================
;
                OUTPUT	slide.com

; --- UART hardware (16C550)
UART_BASE       EQU	0x20             ; base I/O address
UART_RBR        EQU	UART_BASE + 0    ; receive buffer (read)
UART_THR        EQU	UART_BASE + 0    ; transmit holding (write)
UART_IER        EQU	UART_BASE + 1    ; interrupt enable
UART_FCR        EQU	UART_BASE + 2    ; FIFO control (write)
UART_LCR        EQU	UART_BASE + 3    ; line control
UART_MCR        EQU	UART_BASE + 4    ; modem control
UART_LSR        EQU	UART_BASE + 5    ; line status
UART_MSR        EQU	UART_BASE + 6    ; modem status
UART_SCR        EQU	UART_BASE + 7    ; scratch register

; MCR bit masks
MCR_RTS         EQU	0x02             ; request to send
MCR_AFE         EQU	0x20             ; auto flow control enable

; LSR bit masks
LSR_DR          EQU	0x01             ; data ready
LSR_THRE        EQU	0x20             ; transmit holding register empty

; --- Protocol constants -----------------------------------------------------
MAX_RETRIES     EQU	15               ; ~10s before giving up (15 * 660ms per cycle)
SOF             EQU	0x01             ; start of frame
CTRL_ACK        EQU	0x06             ; acknowledge
CTRL_NAK        EQU	0x15             ; negative acknowledge
CTRL_RDY        EQU	0x11             ; ready / handshake
CTRL_FIN        EQU	0x04             ; end of session (renamed from CTRL_EOT)
CTRL_CAN        EQU	0x18             ; cancel (disk error)
CTRL_ENQ        EQU	0x05             ; v0.3 §2: a command frame follows

; v0.3 §3 opcodes
CMD_NOP         EQU	0x00             ; no-op; completes a probe-only exchange
CMD_VOLS        EQU	0x01             ; list available volumes
CMD_DIR         EQU	0x02             ; directory listing

; How long to wait for the command frame after echoing ENQ. Shorter than
; MAX_RETRIES: a client that echoes and then goes quiet is misbehaving, and
; must not be able to park the server here for 30s.
CMD_RETRIES     EQU	5                ; ~10s (5 x ~2s)

; v0.3 §4 reply status codes
ST_OK           EQU	0x00
ST_OPCODE       EQU	0x01             ; unknown / unsupported opcode
ST_OPERAND      EQU	0x02             ; missing or malformed operand
ST_NODRIVE      EQU	0x03             ; drive not present or not selectable
ST_IO           EQU	0x04             ; BDOS/BIOS error during enumeration
ST_BUSY         EQU	0x05             ; cannot service the request now

WIN_SIZE        EQU	4                ; sliding window size
FRAME_SIZE      EQU	1024             ; payload bytes per frame
FLUSH_SIZE      EQU	WIN_SIZE * FRAME_SIZE ; 4KB - flush to disk threshold

; --- CP/M BDOS --------------------------------------------------------------
IOBYTE          EQU	0x0003
BDOS            EQU	0x0005
FCB             EQU	0x005C
DMA_ADDR        EQU	0x0080
CMDTAIL         EQU	0x0080           ; command tail length byte
CMDTEXT         EQU	0x0081           ; command tail text
F_OPEN          EQU	15
F_CLOSE         EQU	16
F_DELETE        EQU	19
F_READ          EQU	20
F_WRITE         EQU	21
F_CREATE        EQU	22
F_SETDMA        EQU	26
F_FSIZE         EQU	35               ; compute file size (sets random record field)
F_SFIRST        EQU	17               ; search for first directory match
F_SNEXT         EQU	18               ; search for next
DRV_SET         EQU	14               ; select disk
DRV_LOGINVEC    EQU	24               ; return login vector
DRV_GET         EQU	25               ; return current disk
F_USERNUM       EQU	32               ; get/set user number

; BIOS jump table. 0x0001 holds the address of the WBOOT entry, which is
; BIOS base + 3; SELDSK is BIOS base + 27, hence WBOOT vector + 24.
; SELDSK is the only way to ask whether a drive exists — BDOS has no such
; call, and its login vector reports drives that have been used, not
; drives that are present.
BIOS_WBOOTV     EQU	0x0001
SELDSK_OFFSET   EQU	24

MAX_DIR_RECS    EQU	250              ; 16 bytes each, must fit RXBUF (4KB)
C_WRITESTR      EQU	9
C_WRITE         EQU	2

; --- Memory layout -----------------------------------------------------------
RXBUF           EQU	0x8000           ; 4KB buffer (used for both send and receive)
RXBUF_END       EQU	RXBUF + FLUSH_SIZE
CRC_TABLE       EQU	0x9000           ; 512 bytes for CRC-16-CCITT lookup table

; ============================================================================
; Entry point
; ============================================================================
                ORG	0x0100           ; CP/M TPA

entry
                ; save/set IOBYTE to keep BIOS off the UART
                LD	HL, IOBYTE
                LD	A, (HL)
                LD	DE, iobyte_saved
                LD	(DE), A

                LD	A, 0b01010110   ; CON=BAT, RDR=PTR, LST=CRT
                LD	(HL), A

                CALL	init_crc_table
                CALL	uart_init

                ; parse command line to determine mode
                CALL	parse_cmdline

                ; branch on mode
                LD	A, (mode)
                OR	A
                JR	NZ, .do_send

                ; --- receive mode ---
                LD	DE, msg_banner_recv
                CALL	print_user_msg
                CALL	recv_session
                JR	.out

.do_send
                ; --- send mode ---
                LD	DE, msg_banner_send
                CALL	print_user_msg
                CALL	send_session
                JR	.out

.out            ; restore original IOBYTE
                LD	HL, iobyte_saved
                LD	DE, IOBYTE
                LD	A, (HL)
                LD	(DE), A

                RST	0               ; warm start back to CP/M

iobyte_saved    DB      0

; ============================================================================
; Command-line parsing
; ============================================================================
; Parse command tail at 0x0080.
; No args or "R" → receive mode (mode=0)
; "S FILE.EXT"   → send mode (mode=1), filename copied to send_fname
; ============================================================================
parse_cmdline
                LD	A, (CMDTAIL)    ; length byte
                OR	A
                RET	Z               ; no args → receive mode (mode already 0)

                ; skip leading spaces
                LD	HL, CMDTEXT
                LD	B, A            ; B = remaining chars
.skip_spaces
                LD	A, (HL)
                CP	' '
                JR	NZ, .got_char
                INC	HL
                DJNZ	.skip_spaces
                RET                 ; all spaces → receive mode

.got_char
                ; check for 'R' or 'r'
                CP	'R'
                RET	Z               ; explicit receive mode
                CP	'r'
                RET	Z

                ; check for 'S' or 's'
                CP	'S'
                JR	Z, .send_mode
                CP	's'
                JR	Z, .send_mode

                ; unknown → receive mode
                RET

.send_mode
                LD	A, 1
                LD	(mode), A
                INC	HL
                DEC	B
                RET	Z               ; "S" with no filename — error handled later

                ; skip spaces before filename
.skip_sp2
                LD	A, (HL)
                CP	' '
                JR	NZ, .got_fname
                INC	HL
                DJNZ	.skip_sp2
                RET                 ; "S " with no filename

.got_fname
                ; copy remaining command tail (all filenames) to safe buffer
                ; HL = pointer to first filename char, B = remaining chars
                LD	DE, cmdtail_buf
.copy_tail
                LD	A, (HL)
                LD	(DE), A
                INC	HL
                INC	DE
                DJNZ	.copy_tail
                XOR	A
                LD	(DE), A         ; null terminate
                ; init pointer to start of buffer
                LD	HL, cmdtail_buf
                LD	(cmdtail_pos), HL
                RET

mode            DB	0               ; 0=receive, 1=send
send_fname      DS	13              ; null-terminated filename for send mode
cmdtail_buf     DS	128             ; saved command tail (filenames)
cmdtail_pos     DW	0               ; current position in cmdtail_buf

; ============================================================================
; next_send_fname — extract next filename from cmdtail_buf into send_fname
; Returns: NZ if filename found, Z if no more filenames
; ============================================================================
next_send_fname
                LD	HL, (cmdtail_pos)
                ; skip spaces
.skip_sp
                LD	A, (HL)
                OR	A
                JR	Z, .no_more     ; end of buffer
                CP	' '
                JR	NZ, .got_name
                INC	HL
                JR	.skip_sp

.got_name
                ; copy filename to send_fname (max 12 chars)
                LD	DE, send_fname
                LD	C, 12
.copy_name
                LD	A, (HL)
                OR	A
                JR	Z, .name_done
                CP	' '
                JR	Z, .name_done
                LD	(DE), A
                INC	HL
                INC	DE
                DEC	C
                JR	NZ, .copy_name
                ; if we hit 12, skip remaining non-space chars
.skip_rest
                LD	A, (HL)
                OR	A
                JR	Z, .name_done
                CP	' '
                JR	Z, .name_done
                INC	HL
                JR	.skip_rest
.name_done
                LD	(cmdtail_pos), HL
                XOR	A
                LD	(DE), A         ; null terminate send_fname
                OR	1               ; set NZ — got a filename
                RET

.no_more
                XOR	A               ; set Z — no more filenames
                RET

; ============================================================================
; UART initialisation
; Set up 16C650: 19200 baud, 8N1, FIFOs enabled, RTS/CTS flow control
; ============================================================================
uart_init
                ; enable DLAB to set baud rate
                LD	A, 0x83           ; DLAB=1, 8 data bits, 1 stop, no parity
                OUT	(UART_LCR), A

                ; divisor for 19200 (1.8432MHz crystal: divisor = 6)
                LD	A, 6
                OUT	(UART_BASE + 0), A
                XOR	A
                OUT	(UART_BASE + 1), A

                ; clear DLAB, keep 8N1
                LD	A, 0x03
                OUT	(UART_LCR), A

                ; enable FIFOs, 8 byte trigger level, clear both
                LD	A, 0x87
                OUT	(UART_FCR), A

                ; enable RTS, enable auto flow control
                LD	A, MCR_RTS | MCR_AFE
                OUT	(UART_MCR), A

                ; disable interrupts (we poll)
                XOR	A
                OUT	(UART_IER), A

                RET

; ============================================================================
; UART primitives
; ============================================================================

; Receive a single byte (blocking)
uart_rx
                IN	A, (UART_LSR)
                BIT	0, A
                JR	Z, uart_rx
                IN	A, (UART_RBR)
                RET

; Receive byte with timeout (~2s at 8MHz)
; Returns: A = byte, carry clear on success; carry set on timeout
; Trashes: B (outer), D (inner)
uart_rx_timeout
                LD	B, 0
.outer
                LD	D, 0
.inner
                IN	A, (UART_LSR)
                BIT	0, A
                JR	NZ, .got_byte
                DEC	D
                JR	NZ, .inner
                DEC	B
                JR	NZ, .outer
                SCF
                RET
.got_byte
                IN	A, (UART_RBR)
                OR	A                 ; clear carry
                RET

; Send a single byte via UART (with ~330ms timeout for CTS)
; A = byte to send.  Preserves all registers.
; On timeout: sets (tx_fail) flag (checked by send_frame)
uart_tx
                PUSH	AF
                PUSH	BC
                LD	B, 0
                LD	C, 0
.wait
                IN	A, (UART_LSR)
                BIT	5, A
                JR	NZ, .ready
                DEC	C
                JR	NZ, .wait
                DEC	B
                JR	NZ, .wait
                ; timeout — set failure flag
                LD	A, 1
                LD	(tx_fail), A
                POP	BC
                POP	AF
                RET
.ready
                POP	BC
                POP	AF
                OUT	(UART_THR), A
                RET

tx_fail         DB	0

; Read a byte if one arrives within ~1.3ms (256 polls at 8MHz).
; Much shorter than uart_rx_timeout — used where we are chasing a byte
; already known to be in flight, not waiting on the peer to decide.
; Returns: carry clear + A = byte, carry set on timeout. Trashes A, B.
uart_rx_brief
                LD	B, 0
.brief_loop
                IN	A, (UART_LSR)
                BIT	0, A
                JR	NZ, .brief_got
                DEC	B
                JR	NZ, .brief_loop
                SCF
                RET
.brief_got
                IN	A, (UART_RBR)
                OR	A                 ; clear carry
                RET

; ============================================================================
; drain_stale_acks — clear leftover ACK/NAK pairs after a reply stream
; without discarding a protocol byte that has already arrived.
;
; A blind uart_flush_rx here loses races. The client's FIN can reach the
; FIFO between the terminator ACK and the flush, and is then swallowed:
; the server sits in .file_loop forever and the session hangs with no
; error anywhere. Measured at roughly 1 in 14 CMD_DIR runs.
;
; Returns: A = 0 if the line went quiet, otherwise the first byte that is
;          not part of a stale ACK/NAK pair. The caller MUST dispatch on
;          it — a UART byte cannot be un-read.
; Trashes A, B.
; ============================================================================
drain_stale_acks
.ds_loop
                IN	A, (UART_LSR)
                BIT	0, A
                JR	Z, .ds_quiet    ; nothing pending — safe to leave
                IN	A, (UART_RBR)
                CP	CTRL_ACK
                JR	Z, .ds_eat_seq
                CP	CTRL_NAK
                JR	Z, .ds_eat_seq
                RET                     ; protocol byte — hand it back

.ds_eat_seq
                ; the sequence byte may still be on the wire (~520us at
                ; 19200), so chase it rather than testing the FIFO once
                CALL	uart_rx_brief
                JR	.ds_loop

.ds_quiet
                XOR	A
                RET

; Flush UART receive FIFO
uart_flush_rx
.flush_loop
                IN	A, (UART_LSR)
                BIT	0, A
                RET	Z
                IN	A, (UART_RBR)
                JR	.flush_loop

; ============================================================================
; Control byte helpers
; ============================================================================

; Send ACK [seq].  A = sequence number.
send_ack
                PUSH	AF
                LD	A, CTRL_ACK
                CALL	uart_tx
                POP	AF
                CALL	uart_tx
                RET

; Send NAK [seq].  A = sequence number.
send_nak
                PUSH	AF
                LD	A, CTRL_NAK
                CALL	uart_tx
                POP	AF
                CALL	uart_tx
                RET

; Send RDY
send_rdy
                LD	A, CTRL_RDY
                CALL	uart_tx
                RET

; Send CAN (cancel) — Z80-initiated. Per v0.2.1 §2:
; sets can_initiated so any incoming CAN is recognised as the peer's
; echo (and not re-echoed); waits briefly for that echo, then drains
; the wire so the next session starts clean.
send_can
                LD	A, 1
                LD	(can_initiated), A
                LD	A, CTRL_CAN
                CALL	uart_tx
                ; wait briefly for peer's echo, then drain
                CALL	uart_rx_timeout
                CALL	uart_flush_rx
                RET

; respond_to_cancel — peer sent CTRL_CAN. Echo back unless we initiated.
; Marks the cancel state so subsequent CAN bytes from either side are no-ops
; (idempotency, v0.2.1 §2.4). Caller should clean up and return to idle.
; Trashes A.
respond_to_cancel
                LD	A, (can_initiated)
                OR	A
                JR	NZ, .skip_echo
                LD	A, CTRL_CAN
                CALL	uart_tx
.skip_echo
                LD	A, 1
                LD	(can_initiated), A
                CALL	uart_flush_rx
                RET

can_initiated   DB	0

; Send FIN
send_fin
                LD	A, CTRL_FIN
                CALL	uart_tx
                RET

; Send wakeup signature ESC ^ S L I D E (v0.2.1 §1).
; Caller: emit at start of recv_session/send_session, before any RDY
; or framed payload. Lets the PC detect SLIDE.COM is alive.
send_wakeup
                LD	HL, wakeup_sig
                LD	B, 7
.loop
                LD	A, (HL)
                CALL	uart_tx
                INC	HL
                DJNZ	.loop
                RET

wakeup_sig      DB	0x1B, 0x5E, 'S', 'L', 'I', 'D', 'E'

; ============================================================================
; recv_control_z80 — wait for PC's ACK/NAK/FIN
; Returns: A = control byte (CTRL_ACK, CTRL_NAK, CTRL_FIN, CTRL_CAN)
;          B = seq (if ACK/NAK)
;          carry set on timeout
; Skips stray RDY and other bytes.
; ============================================================================
recv_control_z80
.wait
                CALL	uart_rx_timeout
                RET	C               ; timeout → carry set

                CP	CTRL_ACK
                JR	Z, .with_seq
                CP	CTRL_NAK
                JR	Z, .with_seq
                CP	CTRL_FIN
                JR	Z, .no_seq
                CP	CTRL_CAN
                JR	Z, .got_can
                ; stray byte (RDY etc) — keep waiting
                JR	.wait

.got_can
                ; v0.2.1 §2: echo CAN back if peer initiated, then return
                ; CTRL_CAN to caller so it can abort cleanly.
                CALL	respond_to_cancel
                LD	A, CTRL_CAN
                OR	A               ; clear carry
                RET

.with_seq
                LD	E, A            ; save control byte (E preserved by uart_rx_timeout)
                CALL	uart_rx_timeout
                JR	C, .timeout_seq
                LD	B, A            ; B = seq
                LD	A, E            ; A = control byte
                OR	A               ; clear carry
                RET

.timeout_seq
                ; got control byte but no seq — treat as timeout
                SCF
                RET

.no_seq
                OR	A               ; clear carry
                RET

; ============================================================================
; Receive frame (used in receive mode)
;
; Expects: SOF SEQ LEN_H LEN_L [PAYLOAD] CRC_H CRC_L
;
; On entry: HL = destination buffer for payload
; Returns:  carry clear = success
;               A = sequence number
;               BC = payload length (0 = end of transfer)
;           carry set = CRC error or timeout
;           D = first byte received (SOF, CTRL_FIN, etc.) if fail_sof
; ============================================================================
recv_frame
                ; clear signal byte; .got_fin / .got_can set it before signalling carry
                LD	D, 0
                ; wait for SOF (or FIN/CAN)
.wait_sof
                CALL	uart_rx_timeout
                JP	C, .fail_sof
                CP	SOF
                JR	Z, .after_sof
                CP	CTRL_FIN
                JR	Z, .got_fin
                CP	CTRL_CAN
                JR	Z, .got_can
                JR	.wait_sof

.got_fin
                ; signal FIN to caller via D=CTRL_FIN, carry set
                LD	D, CTRL_FIN
                SCF
                RET

.got_can
                ; signal CAN to caller via D=CTRL_CAN, carry set (v0.2.1 §2)
                LD	D, CTRL_CAN
                SCF
                RET

.after_sof
                ; --- begin CRC over SEQ+LEN+PAYLOAD ---
                LD	(frame_dst), HL
                LD	HL, 0xFFFF
                LD	(crc_val), HL

                ; receive SEQ
                CALL	uart_rx_timeout
                JP	C, .fail_seq
                LD	(rx_seq), A
                CALL	crc_update_a

                ; receive LEN_H
                CALL	uart_rx_timeout
                JP	C, .fail_lenh
                LD	(rx_len + 1), A
                CALL	crc_update_a

                ; receive LEN_L
                CALL	uart_rx_timeout
                JP	C, .fail_lenl
                LD	(rx_len), A
                CALL	crc_update_a

                ; check for zero-length (end of transfer)
                LD	BC, (rx_len)
                LD	A, B
                OR	C
                JR	Z, .recv_crc

                ; receive payload bytes into (frame_dst), length in BC
                LD	HL, (frame_dst)
                PUSH	HL
                PUSH	BC
.recv_payload
                PUSH	BC
                CALL	uart_rx_timeout
                POP	BC
                JR	C, .payload_err
                LD	(HL), A
                INC	HL
                LD	(frame_dst), HL
                CALL	crc_update_a
                LD	HL, (frame_dst)

                DEC	BC
                LD	A, B
                OR	C
                JR	NZ, .recv_payload
                POP	BC
                POP	HL
                JR	.recv_crc

.payload_err
                POP	BC
                POP	HL
                PUSH    AF
                LD	DE, msg_dbg_tmo
                LD	C, C_WRITESTR
                CALL    BDOS
                POP     AF
                SCF
                RET

                ; receive CRC (high byte first)
.recv_crc
                CALL	uart_rx_timeout
                JR	C, .fail_crch
                LD	(rx_crc + 1), A

                CALL	uart_rx_timeout
                JR	C, .fail_crcl
                LD	(rx_crc), A

                ; compare
                LD	HL, (crc_val)
                LD	DE, (rx_crc)
                OR	A
                SBC	HL, DE
                JR	NZ, .crc_err

                ; success
                LD	A, (rx_seq)
                LD	BC, (rx_len)
                OR	A
                RET

.crc_err
                PUSH	HL
                LD	DE, msg_dbg_crc
                LD	C, C_WRITESTR
                CALL    BDOS
                LD	HL, (crc_val)
                LD	A, H
                CALL    print_hex_a
                LD	A, L
                CALL    print_hex_a
                LD	E, ' '
                LD	C, C_WRITE
                CALL    BDOS
                LD	HL, (rx_crc)
                LD	A, H
                CALL    print_hex_a
                LD	A, L
                CALL    print_hex_a
                LD	E, 13
                LD	C, C_WRITE
                CALL    BDOS
                LD	E, 10
                LD	C, C_WRITE
                CALL    BDOS
                POP     HL
                SCF
                RET

; --- debug fail helpers ---
.dbg_fail_ret
                LD	E, A
                LD	C, C_WRITE
                CALL	BDOS
                SCF
                RET
.fail_sof
                LD	A, 'S'
                JR	.dbg_fail_ret
.fail_seq
                LD	A, '1'
                JR	.dbg_fail_ret
.fail_lenh
                LD	A, '2'
                JR	.dbg_fail_ret
.fail_lenl
                LD	A, '3'
                JR	.dbg_fail_ret
.fail_crch
                LD	A, '4'
                JR	.dbg_fail_ret
.fail_crcl
                LD	A, '5'
                JR	.dbg_fail_ret

; --- frame receive/send temporaries ---
rx_seq          DB	0
rx_len          DW	0
rx_crc          DW	0
crc_val         DW	0
crc_accum       DW	0
frame_dst       DW	0

; ============================================================================
; Receive header frame
; Header payload: null-terminated filename, then 4 bytes file size (LE)
; Returns: carry clear = success, carry set = error
;          D = first byte if FIN was received instead of header
; ============================================================================
recv_header
                LD	HL, RXBUF
                CALL	recv_frame
                JR	C, .hdr_fail

                LD	DE, msg_dbg_ok
                LD	C, C_WRITESTR
                CALL	BDOS

                CALL	parse_filename
                RET

.hdr_fail
                LD	DE, msg_dbg_fail
                LD	C, C_WRITESTR
                CALL	BDOS
                SCF
                RET

; ============================================================================
; Parse filename from RXBUF into CP/M FCB at 0x005C
; Expects null-terminated "FILENAME.EXT" at RXBUF
; ============================================================================
parse_filename
                ; clear FCB
                LD	HL, FCB
                LD	DE, FCB + 1
                LD	BC, 35
                LD	(HL), 0
                LDIR

                ; set drive to default (0)
                XOR	A
                LD	(FCB), A

                ; copy name (up to 8 chars before '.')
                LD	HL, RXBUF
                LD	DE, FCB + 1
                LD	B, 8
.copy_name
                LD	A, (HL)
                OR	A
                JR	Z, .pad_name
                CP	'.'
                JR	Z, .do_ext
                LD	(DE), A
                INC	HL
                INC	DE
                DJNZ	.copy_name
                ; skip to '.' if name was 8 chars
.find_dot
                LD	A, (HL)
                OR	A
                JR	Z, .pad_done
                CP	'.'
                JR	Z, .do_ext
                INC	HL
                JR	.find_dot

.pad_name
                LD	A, ' '
.pad_loop
                LD	(DE), A
                INC	DE
                DJNZ	.pad_loop
                JR	.pad_done

.do_ext
                PUSH	HL
                LD	HL, FCB + 9
                OR	A
                SBC	HL, DE
                LD	B, L
                POP	HL
                LD	A, B
                OR	A
                JR	Z, .ext_start
.pad_n
                LD	A, ' '
                LD	(DE), A
                INC	DE
                DJNZ	.pad_n

.ext_start
                INC	HL
                LD	DE, FCB + 9
                LD	B, 3
.copy_ext
                LD	A, (HL)
                OR	A
                JR	Z, .pad_ext
                LD	(DE), A
                INC	HL
                INC	DE
                DJNZ	.copy_ext
                JR	.pad_done

.pad_ext
                LD	A, ' '
.pad_ext_loop
                LD	(DE), A
                INC	DE
                DJNZ	.pad_ext_loop

.pad_done
                ; store file size (4 bytes after the null terminator)
                LD	HL, RXBUF
.find_null
                LD	A, (HL)
                INC	HL
                OR	A
                JR	NZ, .find_null
                LD	DE, file_size
                LD	BC, 4
                LDIR

                OR	A
                RET

file_size       DW	0, 0

; ============================================================================
; File creation / close (receive mode)
; ============================================================================
create_file
                LD	DE, FCB
                LD	C, F_DELETE
                CALL	BDOS
                LD	DE, FCB
                LD	C, F_CREATE
                CALL	BDOS
                CP	0xFF
                JR	Z, .create_err
                OR	A
                RET
.create_err
                SCF
                RET

close_file
                LD	DE, FCB
                LD	C, F_CLOSE
                CALL	BDOS
                RET

; ============================================================================
; RECEIVE SESSION (multi-file)
; Handshake: wait for sender's RDY, echo RDY back.
; Then loop: receive header → create file → recv_file → close file.
; FIN received instead of SOF → send FIN, exit.
; ============================================================================
recv_session
                ; reset cancel state for fresh session, then emit wakeup
                ; signature so the host knows SLIDE.COM is alive (v0.2.1 §1)
                XOR	A
                LD	(can_initiated), A
                CALL	send_wakeup

                ; --- Handshake: wait for PC's RDY, echo back ---
                LD	E, 15           ; ~30s (15 × ~2s timeout)
.wait_pc
                CALL	uart_rx_timeout
                JR	C, .wait_retry
                CP	CTRL_RDY
                JR	Z, .pc_ready
                CP	CTRL_CAN
                JR	Z, .got_can_hs
                ; not RDY — ignore and keep waiting
                JR	.wait_pc

.wait_retry
                DEC	E
                JR	NZ, .wait_pc
                ; gave up — PC never sent RDY during the handshake window
                LD	DE, msg_err_handshake
                CALL	print_user_msg
                RET

.got_can_hs
                CALL	respond_to_cancel
                LD	DE, msg_cancelled
                CALL	print_user_msg
                RET

.pc_ready
                ; echo RDY back to confirm
                CALL	send_rdy
                CALL	uart_flush_rx

                ; --- File receive loop ---
                ; Wait for SOF (new file header) or FIN (session end).
                ; Discriminate here so BDOS calls can't clobber the byte.
.file_loop
                CALL	uart_rx_timeout
                JR	C, .file_loop   ; timeout — keep waiting
.dispatch                               ; entry with a byte already in A
                CP	CTRL_FIN
                JR	Z, .got_fin
                CP	CTRL_CAN
                JR	Z, .got_can_session
                CP	CTRL_ENQ
                JR	Z, .got_enq
                CP	SOF
                JR	NZ, .file_loop  ; ignore stray bytes

                ; SOF received — receive rest of header frame
                LD	HL, RXBUF
                CALL	recv_frame.after_sof
                JR	C, .err_header

                ; parse filename from header payload
                CALL	parse_filename

                ; create output file
                CALL	create_file
                JR	C, .err_file

                ; ACK header (seq 0)
                LD	A, 0
                CALL	send_ack

                ; receive file data
                CALL	recv_file
                PUSH	AF              ; save error flag

                ; close file (always, even on error)
                CALL	close_file

                POP	AF
                JR	C, .file_error  ; recv_file failed — exit session

                ; reset state for next file
                LD	A, 1
                LD	(expected_seq), A
                LD	HL, RXBUF
                LD	(buf_ptr), HL
                LD	HL, 0
                LD	(buf_used), HL
                XOR	A
                LD	(retry_count), A

                JR	.file_loop

.got_enq
                ; v0.3 §2: a command frame follows. serve_command runs the
                ; whole exchange and normally leaves us back here; it only
                ; reports FIN or CAN if one arrived instead of a command.
                CALL	serve_command
                OR	A
                JP	Z, .file_loop
                ; Non-zero means serve_command handed back a byte it read but
                ; must not discard — FIN, CAN, a following SOF, even another
                ; ENQ. Dispatch it exactly as if the file loop had read it.
                JP	.dispatch

.got_fin
                ; Print the session-complete message BEFORE echoing FIN.
                ; PC closes the serial port the instant it sees our FIN echo
                ; (slide-py's `ser.close()` immediately after the FIN exchange),
                ; which drops RTS → CTS goes low at our UART → AFE blocks
                ; transmission → BIOS console output (no timeout) would hang
                ; forever on LSR_THRE. PC's recv_control already skips
                ; non-control bytes, so the text is harmless on the wire.
                LD	DE, msg_done_session
                CALL	print_user_msg

                ; echo FIN back
                CALL	send_fin
                RET

.got_can_session
                CALL	respond_to_cancel
                LD	DE, msg_cancelled
                CALL	print_user_msg
                RET

.file_error
                ; recv_file already printed error and sent CAN — just exit
                RET

.err_header
                LD	DE, msg_err_hdr
                CALL	print_user_msg
                CALL	send_can
                RET

.err_file
                LD	DE, msg_err_file
                CALL	print_user_msg
                CALL	send_can
                RET

; ============================================================================
; serve_command — run one v0.3 §2 command exchange.
;
; Entry: CTRL_ENQ has just been consumed by the caller's file loop.
; Exit:  A = 0        → handled; caller returns to its file loop
;        A = CTRL_FIN → peer sent FIN instead of a command frame
;        A = CTRL_CAN → peer cancelled (already echoed)
;
; Wire sequence (v0.3 §2, §4):
;   <- ENQ                      (consumed by the caller)
;   -> ENQ '0' '0' '0' '1'      capability echo + command-set version
;   <- SOF + command frame, seq 0
;   -> ACK 0
;   -> status frame seq 1, record frame(s), zero-length terminator
;   <- ACK
;
; Replies can exceed WIN_SIZE frames now that CMD_DIR enumerates a real
; directory, so record frames go out through send_reply_frame, which
; absorbs the client's window ACKs.
;
; Deliberately silent: no print_user_msg during the exchange. Console
; output in receive mode goes out this same UART, and would show up as
; stray bytes in the client's reply stream.
; ============================================================================
serve_command
                CALL	send_enq_echo

                ; Wait for the command frame. CRC failures are retried here
                ; rather than in the file loop: a retransmitted command frame
                ; arriving there would be parsed as a file header.
                LD	A, CMD_RETRIES
                LD	(cmd_retry), A

.wait_cmd
                CALL	uart_rx_timeout
                JR	C, .retry       ; timeout — count it and keep waiting
                CP	CTRL_FIN
                JR	Z, .got_fin
                CP	CTRL_CAN
                JR	Z, .got_can
                CP	CTRL_ENQ
                JR	Z, .dup_enq     ; client retried the probe (§2 idempotency)
                CP	SOF
                JR	NZ, .wait_cmd   ; stray byte — not worth a retry

                LD	HL, RXBUF
                CALL	recv_frame.after_sof
                JR	NC, .got_cmd

                ; bad frame — NAK seq 0 and let the client retransmit
                XOR	A
                CALL	send_nak

.retry
                LD	A, (cmd_retry)
                DEC	A
                LD	(cmd_retry), A
                JR	NZ, .wait_cmd
                XOR	A               ; gave up — back to the file loop
                RET

.dup_enq
                CALL	send_enq_echo
                JR	.retry

.got_fin
                LD	A, CTRL_FIN
                RET

.got_can
                CALL	respond_to_cancel
                LD	A, CTRL_CAN
                RET

.got_cmd
                ; BC = payload length, RXBUF = payload. ACK the frame first
                ; (§4), then decide what to answer with.
                PUSH	BC
                XOR	A
                CALL	send_ack        ; ACK 0
                POP	BC

                LD	A, B
                OR	C
                JR	Z, .err_operand ; empty payload — no opcode in it

                LD	A, (RXBUF)      ; opcode
                CP	CMD_NOP
                JR	Z, .do_nop
                CP	CMD_VOLS
                JR	Z, .do_vols
                CP	CMD_DIR
                JR	Z, .do_dir

                LD	A, ST_OPCODE
                JR	.status_only

.do_nop
                ; v0.3 §3: completes a probe-only exchange. The echo has
                ; already told the client what it wanted to know; this just
                ; releases the server from command mode without a listing.
                LD	A, ST_OK
                JR	.status_only

.err_operand
                LD	A, ST_OPERAND

.status_only
                ; error reply: status frame (seq 1) then terminator (§4)
                LD	(cmd_status), A
                LD	HL, cmd_status
                LD	BC, 1
                LD	A, 1
                CALL	send_frame
                LD	A, 2
                JP	.terminate

.do_vols
                CALL	enum_volumes
                ; One frame carries the status byte and the single 5-byte
                ; record together — §4 permits this when they fit.
                LD	HL, vols_buf
                LD	BC, VOLS_BUF_LEN
                LD	A, 1
                CALL	send_frame
                LD	A, 2
                JP	.terminate

.do_dir
                ; Operands: [opcode][drive][user] and optionally an 11-byte
                ; match pattern. Copy them out before enum_dir runs — it
                ; builds its records in RXBUF, over this very payload.
                LD	A, B
                OR	A
                JP	NZ, .err_operand
                LD	A, C
                CP	3
                JR	Z, .dir_matchall
                CP	14
                JP	NZ, .err_operand
                LD	HL, RXBUF + 3
                LD	DE, dir_pattern
                LD	BC, 11
                LDIR
                JR	.dir_operands
.dir_matchall
                LD	HL, dir_pattern
                LD	B, 11
                LD	A, '?'
.dir_fillpat
                LD	(HL), A
                INC	HL
                DJNZ	.dir_fillpat
.dir_operands
                LD	A, (RXBUF + 1)
                LD	(dir_drive), A
                LD	A, (RXBUF + 2)
                LD	(dir_user), A

                CALL	enum_dir
                JP	C, .status_only ; A = status code

                ; Record count is not known up front, so send the status
                ; frame alone and stream records behind it (§4).
                LD	A, ST_OK
                LD	(cmd_status), A
                LD	HL, cmd_status
                LD	BC, 1
                LD	A, 1
                CALL	send_reply_frame

                LD	A, 2
                LD	(reply_seq), A
                LD	HL, RXBUF
                LD	(reply_ptr), HL
                LD	HL, (dir_bytes)
                LD	(reply_left), HL

.dir_frames
                LD	HL, (reply_left)
                LD	A, H
                OR	L
                JR	Z, .dir_frames_done

                ; chunk = min(left, FRAME_SIZE); records never split because
                ; FRAME_SIZE is a whole number of 16-byte records
                LD	DE, FRAME_SIZE
                OR	A
                SBC	HL, DE
                JR	C, .dir_last
                LD	(reply_left), HL
                LD	BC, FRAME_SIZE
                JR	.dir_send
.dir_last
                LD	HL, (reply_left)
                LD	B, H
                LD	C, L
                LD	HL, 0
                LD	(reply_left), HL
.dir_send
                LD	HL, (reply_ptr)
                PUSH	BC
                LD	A, (reply_seq)
                CALL	send_reply_frame
                POP	BC
                LD	HL, (reply_ptr)
                ADD	HL, BC
                LD	(reply_ptr), HL
                LD	A, (reply_seq)
                INC	A
                LD	(reply_seq), A
                JR	.dir_frames

.dir_frames_done
                LD	A, (reply_seq)

.terminate
                ; zero-length frame ends the reply stream; A = its seq
                LD	HL, 0
                LD	BC, 0
                CALL	send_frame

                ; wait for the client's ACK of the terminator
                CALL	recv_control_z80
                JR	C, .done        ; no ACK — client gone; still clean up
                CP	CTRL_CAN
                JR	Z, .can_after

.done
                ; v0.3 §9: clear stale ACK/NAK pairs before returning to the
                ; file loop — in receive mode this is the only place ACKs
                ; can be left queued, and a leftover ACK's sequence byte
                ; would read as SOF or FIN there.
                ;
                ; Selective, NOT a blind flush: uart_flush_rx here ate the
                ; client's FIN when it landed between the terminator ACK and
                ; the drain, hanging the session about 1 run in 14. Anything
                ; drain_stale_acks could not legitimately eat comes back in A
                ; and the caller dispatches on it.
                CALL	drain_stale_acks
                RET

.can_after
                CALL	uart_flush_rx
                LD	A, CTRL_CAN
                RET

; ----------------------------------------------------------------------------
; send_reply_frame — send one reply frame and absorb the window ACK.
;
; A = seq, HL = payload, BC = length. The client ACKs every WIN_SIZE frames
; exactly as it does for file data, so a reply longer than WIN_SIZE frames
; puts ACKs on the wire mid-stream. Left unread they would pile up behind
; the terminator ACK and confuse the drain. Short replies never reach a
; boundary and this costs nothing.
; ----------------------------------------------------------------------------
send_reply_frame
                PUSH	AF
                CALL	send_frame
                POP	AF
                AND	WIN_SIZE - 1
                RET	NZ
                CALL	recv_control_z80
                RET

; ----------------------------------------------------------------------------
; send_enq_echo — v0.3 §2 capability echo: CTRL_ENQ then the 4-byte VER.
; Unconditional and stateless. Trashes A, B, HL.
; ----------------------------------------------------------------------------
send_enq_echo
                LD	A, CTRL_ENQ
                CALL	uart_tx
                LD	HL, cmdset_ver
                LD	B, 4
.ve_loop
                LD	A, (HL)
                CALL	uart_tx
                INC	HL
                DJNZ	.ve_loop
                RET

cmdset_ver      DB	'0', '0', '0', '1'  ; v0.3 §2 VER: major 00, minor 01
cmd_status      DB	0
cmd_retry       DB	0

; ============================================================================
; call_seldsk — BIOS SELDSK for the drive in C.
;
; Returns HL = the drive's DPH address, or HL = 0 if the drive is absent.
; That zero is the only presence test CP/M offers: BDOS has no "does this
; drive exist" call, and DRV_LOGINVEC answers a different question.
;
; Reached through the WBOOT vector rather than a hardcoded address, so it
; follows whatever BIOS this machine booted. Trashes A, DE, HL.
; ============================================================================
call_seldsk
                LD	HL, (BIOS_WBOOTV)
                LD	DE, SELDSK_OFFSET
                ADD	HL, DE
                LD	(seldsk_ptr), HL
                LD	E, 0            ; bit 0 clear = log in as if new
                CALL	.via_ptr
                RET
.via_ptr
                LD	HL, (seldsk_ptr)
                JP	(HL)            ; SELDSK's RET lands back in call_seldsk

; ============================================================================
; enum_volumes — fill vols_buf for CMD_VOLS (v0.3 §5).
;
; Probes all 16 drives with BIOS SELDSK. Going behind BDOS's back leaves it
; pointing at the last drive probed, so the current drive is saved first and
; restored afterwards — otherwise every later file operation lands on the
; wrong disk.
; ============================================================================
enum_volumes
                LD	C, DRV_GET
                CALL	BDOS
                LD	(saved_drive), A
                LD	(vols_current), A

                LD	C, DRV_LOGINVEC
                CALL	BDOS            ; HL = login vector
                LD	(vols_logged), HL

                LD	HL, 0           ; present bitmap
                LD	DE, 1           ; bit mask for the drive under test
                LD	B, 0            ; drive number
.probe_loop
                PUSH	HL
                PUSH	DE
                PUSH	BC
                LD	C, B
                CALL	call_seldsk
                LD	A, H
                OR	L               ; Z set = HL was 0 = drive absent
                POP	BC
                POP	DE
                POP	HL              ; POPs do not disturb the flags
                JR	Z, .absent
                LD	A, H
                OR	D
                LD	H, A
                LD	A, L
                OR	E
                LD	L, A
.absent
                EX	DE, HL          ; mask <<= 1
                ADD	HL, HL
                EX	DE, HL
                INC	B
                LD	A, B
                CP	16
                JR	NZ, .probe_loop

                LD	(vols_present), HL

                ; put BDOS back where it was
                LD	A, (saved_drive)
                LD	E, A
                LD	C, DRV_SET
                CALL	BDOS
                RET

vols_buf        DB	ST_OK
vols_present    DW	0               ; LE bitmap, bit n = drive n selectable
vols_logged     DW	0               ; LE bitmap, BDOS login vector
vols_current    DB	0
VOLS_BUF_LEN    EQU	$ - vols_buf

; ============================================================================
; enum_dir — build 16-byte CMD_DIR records into RXBUF (v0.3 §5).
;
; Entry: dir_drive = 0..15 or 0xFF for current, dir_user likewise,
;        dir_pattern = 11-byte FCB-style match.
; Exit:  carry clear, dir_bytes = record bytes written
;        carry set, A = status code (§4)
;
; Two passes, because they cannot be interleaved: CP/M keeps ONE global
; search state, so any other BDOS disk call between F_SNEXT calls destroys
; it. Pass 1 walks the directory collecting names; pass 2 asks F_FSIZE for
; each size afterwards.
;
; Only extent 0 is matched (search FCB ex = 0), so a file larger than 16KB
; appears once rather than once per extent.
; ============================================================================
enum_dir
                ; --- save the state we are about to disturb ---
                LD	C, DRV_GET
                CALL	BDOS
                LD	(saved_drive), A

                LD	E, 0xFF         ; 0xFF = interrogate, do not set
                LD	C, F_USERNUM
                CALL	BDOS
                LD	(saved_user), A

                ; --- select the requested drive ---
                LD	A, (dir_drive)
                CP	0xFF
                JR	Z, .drive_ok    ; current drive, nothing to do
                CP	16
                JP	NC, .bad_operand

                ; presence first: selecting an absent drive prompts BDOS to
                ; ask the user to fix the disk, which nobody is there to do
                LD	C, A
                PUSH	BC
                CALL	call_seldsk
                POP	BC
                LD	A, H
                OR	L
                JP	Z, .no_drive

                LD	A, (dir_drive)
                LD	E, A
                LD	C, DRV_SET
                CALL	BDOS
.drive_ok
                ; --- select the requested user ---
                LD	A, (dir_user)
                CP	0xFF
                JR	Z, .user_ok
                CP	16
                JP	NC, .bad_operand
                LD	E, A
                LD	C, F_USERNUM
                CALL	BDOS
.user_ok
                ; --- DMA for the directory buffer ---
                LD	DE, DMA_ADDR
                LD	C, F_SETDMA
                CALL	BDOS

                ; --- build the search FCB ---
                CALL	build_search_fcb

                LD	HL, RXBUF
                LD	(dir_ptr), HL
                XOR	A
                LD	(dir_count), A

                LD	DE, dir_fcb
                LD	C, F_SFIRST
                CALL	BDOS
.search_loop
                CP	0xFF
                JR	Z, .search_done

                CALL	store_dir_entry ; A = directory code 0..3
                JR	C, .search_done ; record buffer full

                LD	DE, dir_fcb
                LD	C, F_SNEXT
                CALL	BDOS
                JR	.search_loop

.search_done
                ; --- pass 2: sizes, now that the search state is spent ---
                CALL	fill_dir_sizes

                ; dir_bytes = dir_count * 16
                LD	A, (dir_count)
                LD	L, A
                LD	H, 0
                ADD	HL, HL
                ADD	HL, HL
                ADD	HL, HL
                ADD	HL, HL
                LD	(dir_bytes), HL

                CALL	restore_drive_user
                OR	A               ; clear carry — success
                RET

.no_drive
                LD	A, ST_NODRIVE
                JR	.fail
.bad_operand
                LD	A, ST_OPERAND
.fail
                PUSH	AF
                CALL	restore_drive_user
                POP	AF
                SCF
                RET

; restore_drive_user — put the current drive and user back as we found them.
restore_drive_user
                LD	A, (saved_user)
                LD	E, A
                LD	C, F_USERNUM
                CALL	BDOS
                LD	A, (saved_drive)
                LD	E, A
                LD	C, DRV_SET
                CALL	BDOS
                RET

; build_search_fcb — zeroed FCB, current drive, dir_pattern, extent 0.
build_search_fcb
                LD	HL, dir_fcb
                LD	B, 36
                XOR	A
.clr
                LD	(HL), A
                INC	HL
                DJNZ	.clr

                LD	HL, dir_pattern
                LD	DE, dir_fcb + 1
                LD	BC, 11
                LDIR
                RET                     ; drive byte and ex already 0

; ============================================================================
; store_dir_entry — append one 16-byte record for the match just found.
; Entry: A = directory code (0..3); the entry is at DMA_ADDR + A*32.
; Exit:  carry set if the record buffer is full.
; ============================================================================
store_dir_entry
                ; HL = DMA_ADDR + A*32
                AND	0x03
                LD	L, A
                LD	H, 0
                ADD	HL, HL
                ADD	HL, HL
                ADD	HL, HL
                ADD	HL, HL
                ADD	HL, HL
                LD	DE, DMA_ADDR
                ADD	HL, DE

                LD	A, (dir_count)
                CP	MAX_DIR_RECS
                JR	NC, .full

                PUSH	HL              ; HL = directory entry
                LD	DE, (dir_ptr)

                LD	A, (HL)         ; byte 0 = user number
                LD	(DE), A
                INC	DE
                INC	HL

                ; 11 name bytes, attribute bits masked off
                LD	B, 11
.name_loop
                LD	A, (HL)
                AND	0x7F
                LD	(DE), A
                INC	HL
                INC	DE
                DJNZ	.name_loop

                ; attributes live in the high bits of name bytes 8, 9, 10
                POP	HL
                PUSH	HL
                LD	BC, 9           ; entry + 9 = name[8] = t1'
                ADD	HL, BC
                LD	A, (HL)
                RLCA                    ; bit 7 -> bit 0
                AND	0x01            ; R/O
                LD	C, A
                INC	HL
                LD	A, (HL)
                RLCA
                AND	0x01
                ADD	A, A            ; SYS -> bit 1
                OR	C
                LD	C, A
                INC	HL
                LD	A, (HL)
                RLCA
                AND	0x01
                ADD	A, A
                ADD	A, A            ; archive -> bit 2
                OR	C
                LD	(DE), A
                INC	DE

                XOR	A               ; size filled in by pass 2
                LD	(DE), A
                INC	DE
                LD	(DE), A
                INC	DE
                LD	(DE), A
                INC	DE

                LD	(dir_ptr), DE
                POP	HL

                LD	A, (dir_count)
                INC	A
                LD	(dir_count), A
                OR	A               ; clear carry
                RET
.full
                SCF
                RET

; ============================================================================
; fill_dir_sizes — pass 2. For every collected record, ask F_FSIZE for the
; file size and write it into the record's 3-byte size field.
;
; Must run after the search completes: F_FSIZE is a BDOS disk call and would
; destroy the search state if interleaved with F_SNEXT.
; ============================================================================
fill_dir_sizes
                LD	A, (dir_count)
                OR	A
                RET	Z
                LD	B, A
                LD	HL, RXBUF
.size_loop
                PUSH	BC
                PUSH	HL

                ; FCB for this record: drive 0 (current), name, extent 0
                PUSH	HL
                LD	HL, dir_fcb
                LD	B, 36
                XOR	A
.clr2
                LD	(HL), A
                INC	HL
                DJNZ	.clr2
                POP	HL

                PUSH	HL
                INC	HL              ; skip the user byte
                LD	DE, dir_fcb + 1
                LD	BC, 11
                LDIR
                POP	HL

                PUSH	HL
                LD	DE, dir_fcb
                LD	C, F_FSIZE
                CALL	BDOS
                POP	HL

                ; record + 13..15 = FCB + 33..35 (24-bit LE record count)
                PUSH	HL
                LD	DE, 13
                ADD	HL, DE
                LD	A, (dir_fcb + 33)
                LD	(HL), A
                INC	HL
                LD	A, (dir_fcb + 34)
                LD	(HL), A
                INC	HL
                LD	A, (dir_fcb + 35)
                LD	(HL), A
                POP	HL

                LD	DE, 16
                ADD	HL, DE
                POP	DE              ; discard the saved HL (we advanced it)
                POP	BC
                DJNZ	.size_loop
                RET

seldsk_ptr      DW	0
reply_seq       DB	0
reply_ptr       DW	0
reply_left      DW	0
saved_drive     DB	0
saved_user      DB	0
dir_drive       DB	0xFF
dir_user        DB	0xFF
dir_count       DB	0
dir_ptr         DW	0
dir_bytes       DW	0
dir_pattern     DS	11
dir_fcb         DS	36

; ============================================================================
; Main file receive loop (single file, called by recv_session)
; Receives frames with sliding window, buffers in RAM, flushes to disk
; ============================================================================
recv_file
                LD	A, 1
                LD	(expected_seq), A
                LD	HL, RXBUF
                LD	(buf_ptr), HL
                LD	HL, 0
                LD	(buf_used), HL

.recv_loop
                LD	HL, (buf_ptr)
                CALL	recv_frame
                JR	C, .handle_error

                ; save seq before zero-length check clobbers A
                LD	D, A

                ; reset retry counter
                XOR	A
                LD	(retry_count), A

                ; check for EOF (zero-length frame)
                LD	A, B
                OR	C
                JP	Z, .end_of_file

                ; verify sequence number
                LD	A, (expected_seq)
                CP	D
                JR	NZ, .seq_error

                ; advance buffer pointer
                LD	HL, (buf_ptr)
                ADD	HL, BC
                LD	(buf_ptr), HL

                ; track total buffered
                LD	HL, (buf_used)
                ADD	HL, BC
                LD	(buf_used), HL

                ; increment expected sequence
                LD	A, (expected_seq)
                INC	A
                LD	(expected_seq), A

                ; flush before ACK
                LD	HL, (buf_used)
                LD	DE, FLUSH_SIZE
                OR	A
                SBC	HL, DE
                JR	C, .no_flush

                CALL	flush_to_disk
                JR	C, .disk_error
                LD	HL, RXBUF
                LD	(buf_ptr), HL
                LD	HL, 0
                LD	(buf_used), HL

.no_flush
                ; ACK every WIN_SIZE frames
                LD	A, (expected_seq)
                DEC	A
                AND	WIN_SIZE - 1
                JR	NZ, .recv_loop

                LD	A, (expected_seq)
                DEC	A
                CALL	send_ack
                JR	.recv_loop

.handle_error
                ; v0.2.1 §2: recv_frame signals peer-initiated cancel via D=CTRL_CAN.
                ; CAN was already echoed by recv_frame's caller path — just clean up.
                LD	A, D
                CP	CTRL_CAN
                JP	Z, .got_can_recv

                LD	A, (retry_count)
                INC	A
                LD	(retry_count), A
                CP	MAX_RETRIES
                JR	NC, .abort

                LD	A, (expected_seq)
                CALL	send_nak
                JR	.recv_loop

.got_can_recv
                CALL	respond_to_cancel
                LD	DE, msg_cancelled
                CALL	print_user_msg
                SCF                  ; signal error to caller
                RET

.seq_error
                LD	A, (expected_seq)
                CALL	send_nak
                JP	.recv_loop

.abort
                LD	DE, msg_err_abort
                CALL	print_user_msg
                SCF                  ; signal error to caller
                RET

.disk_error
                CALL	send_can
                SCF                  ; signal error to caller
                RET

.end_of_file
                LD	HL, (buf_used)
                LD	A, H
                OR	L
                JR	Z, .eof_ack
                CALL	flush_to_disk
                JR	C, .disk_error

.eof_ack
                LD	A, (expected_seq)
                CALL	send_ack

                LD	DE, msg_done
                CALL	print_user_msg
                OR	A                ; clear carry = success
                RET

; --- recv state ---
expected_seq    DB	0
buf_ptr         DW	RXBUF
buf_used        DW	0
retry_count     DB	0

; ============================================================================
; Flush buffer to disk via CP/M sequential writes
; ============================================================================
flush_to_disk
                ; zero-pad the final partial record: F_WRITE always emits a
                ; full 128-byte sector, so the unused tail past EOF would
                ; otherwise leak stale RXBUF data to disk. Only fires on the
                ; EOF flush — intermediate flushes are exactly FLUSH_SIZE.
                LD	HL, (buf_used)
                LD	A, L
                AND	0x7F             ; buf_used mod 128 (bytes in final record)
                JR	Z, .no_pad       ; exact multiple -> nothing to pad
                LD	C, A
                LD	A, 128
                SUB	C
                LD	B, A             ; B = 128 - (buf_used mod 128) = pad count
                LD	DE, RXBUF
                ADD	HL, DE           ; HL = RXBUF + buf_used = first pad byte
.pad_loop
                LD	(HL), 0
                INC	HL
                DJNZ	.pad_loop
.no_pad
                LD	HL, RXBUF
                LD	DE, (buf_used)

.write_loop
                LD	A, D
                OR	E
                RET	Z

                ; set DMA
                PUSH	DE
                PUSH	HL
                LD	D, H
                LD	E, L
                LD	C, F_SETDMA
                CALL	BDOS
                POP	HL
                POP	DE

                ; write one 128-byte record
                PUSH	HL
                PUSH	DE
                LD	A, (FCB + 32)
                PUSH	AF
                LD	DE, FCB
                LD	C, F_WRITE
                CALL	BDOS
                POP	BC
                POP	DE
                POP	HL

                OR	A
                JR	NZ, .write_err
                LD	A, (FCB + 32)
                CP	B
                JR	Z, .write_err

                LD	BC, 128
                ADD	HL, BC

                EX	DE, HL
                OR	A
                SBC	HL, BC
                JR	C, .write_done
                EX	DE, HL
                JR	.write_loop

.write_done
                OR	A
                RET

.write_err
                LD	DE, msg_err_disk
                CALL	print_user_msg
                SCF
                RET

; ============================================================================
; SEND SESSION (multi-file from command line)
; Handshake: Z80 (sender) sends RDY, waits for PC's RDY echo.
; Then: for each file: open → send_file_tx → close. After all files: FIN exchange.
; ============================================================================
send_session
                ; reset cancel state and emit wakeup signature (v0.2.1 §1)
                XOR	A
                LD	(can_initiated), A
                CALL	send_wakeup

                ; --- Handshake: sender sends RDY, waits for PC's RDY echo ---
                LD	E, 15           ; ~30s
.wait_pc
                CALL	send_rdy
                CALL	uart_rx_timeout
                JR	C, .wait_retry
                CP	CTRL_RDY
                JR	Z, .pc_ready
                CP	CTRL_CAN
                JR	Z, .got_can_hs
                ; not RDY — keep trying
                JR	.wait_pc

.wait_retry
                DEC	E
                JR	NZ, .wait_pc
                ; gave up
                LD	DE, msg_err_nopc
                CALL	print_user_msg
                RET

.got_can_hs
                CALL	respond_to_cancel
                LD	DE, msg_cancelled
                CALL	print_user_msg
                RET

.pc_ready
                CALL	uart_flush_rx

                ; --- File loop: send each file from command line ---
.next_file
                CALL	next_send_fname
                JR	Z, .all_sent    ; no more filenames

                ; --- Set up FCB from send_fname ---
                ; Copy send_fname into RXBUF so parse_filename can use it
                LD	HL, send_fname
                LD	DE, RXBUF
                LD	BC, 13
                LDIR

                CALL	parse_filename

                ; print filename
                LD	DE, msg_sending
                CALL	print_user_msg
                CALL	print_fcb_name

                ; open file
                LD	DE, FCB
                LD	C, F_OPEN
                CALL	BDOS
                CP	0xFF
                JR	Z, .err_open

                ; compute file size via BDOS F_FSIZE
                CALL	compute_file_size

                ; send file
                CALL	send_file_tx

                ; close file
                CALL	close_file

                ; v0.2.1 §2: if cancel happened during this file, abort the
                ; whole session — don't try to send subsequent files into a
                ; peer that has returned to idle.
                LD	A, (can_initiated)
                OR	A
                JR	NZ, .session_cancelled

                JR	.next_file      ; loop for next file

.session_cancelled
                LD	DE, msg_cancelled
                CALL	print_user_msg
                RET

.all_sent
                ; Print BEFORE the FIN exchange — see the comment in
                ; recv_session .got_fin. Once PC sees our FIN, it sends an
                ; echo and closes the port, dropping RTS and stalling any
                ; subsequent BIOS console output via AFE/CTS.
                LD	DE, msg_done_session
                CALL	print_user_msg

                ; --- Send FIN, wait for echo ---
                CALL	send_fin
                ; wait for FIN echo (brief timeout)
                CALL	recv_control_z80
                ; don't care about result — we're done
                RET

.err_open
                LD	DE, msg_err_open
                CALL	print_user_msg
                ; continue with remaining files
                JR	.next_file

; ============================================================================
; Compute file size using BDOS function 35
; Sets file_size (4 bytes) = random record count × 128
; FCB must be set up with filename already opened.
; ============================================================================
compute_file_size
                ; zero random record field (FCB+33..35)
                XOR	A
                LD	(FCB + 33), A
                LD	(FCB + 34), A
                LD	(FCB + 35), A

                ; BDOS F_FSIZE sets FCB+33..35 to record count
                LD	DE, FCB
                LD	C, F_FSIZE
                CALL	BDOS

                ; file_size = record_count << 7  (record_count * 128)
                ; record_count is 3 bytes: R0=FCB+33, R1=FCB+34, R2=FCB+35
                ; After <<7:
                ;   byte 0 = (R0 & 1) << 7
                ;   byte 1 = (R0 >> 1) | ((R1 & 1) << 7)
                ;   byte 2 = (R1 >> 1) | ((R2 & 1) << 7)
                ;   byte 3 = R2 >> 1

                ; bytes 0-1 via HL shift
                LD	A, (FCB + 33)
                LD	L, A
                LD	A, (FCB + 34)
                LD	H, A
                LD	B, 7
.shift
                ADD	HL, HL
                DJNZ	.shift
                LD	(file_size), HL

                ; byte 2 = (R1 >> 1) | ((R2 & 1) << 7)
                LD	A, (FCB + 34)
                SRL	A               ; R1 >> 1
                LD	B, A
                LD	A, (FCB + 35)
                RRCA                ; bit 0 → bit 7
                AND	0x80
                OR	B
                LD	(file_size + 2), A

                ; byte 3 = R2 >> 1
                LD	A, (FCB + 35)
                SRL	A
                LD	(file_size + 3), A

                ; reset sequential record to 0 for reading
                XOR	A
                LD	(FCB + 32), A

                RET

; ============================================================================
; send_file_tx — send one file over the wire
; FCB must be open, file_size must be set.
; Uses RXBUF as read buffer (same 4KB region).
; ============================================================================
send_file_tx
                ; --- Send header frame ---
                ; build header payload in RXBUF: "FILENAME.EXT\0" + 4-byte LE size
                CALL	build_header_payload
                ; BC = payload length (returned by build_header_payload)
                ; send as frame with seq 0
                LD	HL, RXBUF
                XOR	A               ; seq = 0
                CALL	send_frame
                ; wait for ACK
                CALL	recv_control_z80
                JP	C, .tx_abort
                CP	CTRL_ACK
                JP	NZ, .tx_abort

                ; --- Main send loop ---
                LD	A, 1
                LD	(tx_seq), A
                XOR	A
                LD	(tx_retry), A
                LD	(tx_eof), A

.read_loop
                ; read up to FLUSH_SIZE bytes from disk into RXBUF
                CALL	read_from_disk
                ; BC = bytes read, carry set if EOF reached
                LD	(tx_chunk_len), BC

                ; save EOF flag in variable (no stack juggling)
                LD	A, 0
                JR	NC, .no_eof_yet
                LD	A, 1
.no_eof_yet
                LD	(tx_eof), A

                ; nothing read? just send EOF frame
                LD	A, B
                OR	C
                JR	Z, .send_eof

                ; --- Send window of frames from RXBUF ---
.send_window
                CALL	send_window_from_buf

                ; if EOF, append zero-length frame after data
                LD	A, (tx_eof)
                OR	A
                JR	Z, .wait_ack

                LD	HL, RXBUF
                LD	A, (tx_seq)
                LD	BC, 0
                CALL	send_frame

.wait_ack
                ; wait for ACK/NAK
                CALL	recv_control_z80
                JR	C, .tx_timeout
                CP	CTRL_NAK
                JR	Z, .handle_nak
                CP	CTRL_ACK
                JR	NZ, .tx_abort

                ; ACK received — reset retries
                XOR	A
                LD	(tx_retry), A

                ; done if EOF was set
                LD	A, (tx_eof)
                OR	A
                JR	NZ, .tx_done
                JR	.read_loop

.send_eof
                ; nothing read — just send EOF frame
                LD	HL, RXBUF
                LD	A, (tx_seq)
                LD	BC, 0
                CALL	send_frame
                LD	A, 1
                LD	(tx_eof), A
                JR	.wait_ack

.handle_nak
                LD	A, (tx_retry)
                INC	A
                LD	(tx_retry), A
                CP	MAX_RETRIES
                JR	NC, .tx_abort
                ; rewind tx_seq to start of this window and retransmit
                LD	A, (tx_win_start)
                LD	(tx_seq), A
                JR	.send_window

.tx_timeout
                ; if uart_tx failed, PC is gone — abort immediately
                LD	A, (tx_fail)
                OR	A
                JR	NZ, .tx_abort
                LD	A, (tx_retry)
                INC	A
                LD	(tx_retry), A
                CP	MAX_RETRIES
                JR	NC, .tx_abort
                LD	A, (tx_win_start)
                LD	(tx_seq), A
                JR	.send_window

.tx_abort
                LD	DE, msg_err_abort
                CALL	print_user_msg
                RET

.tx_done
                LD	DE, msg_done
                CALL	print_user_msg
                RET

tx_seq          DB	0
tx_retry        DB	0
tx_eof          DB	0
tx_chunk_len    DW	0
tx_win_start    DB	0

; ============================================================================
; send_window_from_buf — send frames from RXBUF covering tx_chunk_len bytes
; Updates tx_seq.  Sends up to WIN_SIZE frames at a time, waits for ACK
; between windows if chunk is larger than one window.
; ============================================================================
send_window_from_buf
                LD	A, (tx_seq)
                LD	(tx_win_start), A

                LD	HL, RXBUF
                LD	DE, (tx_chunk_len)

.frame_loop
                ; bail out if uart_tx failed (PC disconnected)
                LD	A, (tx_fail)
                OR	A
                RET	NZ

                ; any data left?
                LD	A, D
                OR	E
                RET	Z

                ; determine this frame's payload size: min(FRAME_SIZE, remaining)
                PUSH	HL
                PUSH	DE
                LD	BC, FRAME_SIZE
                ; if DE < FRAME_SIZE, use DE
                EX	DE, HL          ; HL = remaining
                OR	A
                SBC	HL, BC          ; HL = remaining - FRAME_SIZE
                JR	NC, .use_frame_size
                ; remaining < FRAME_SIZE
                ADD	HL, BC          ; restore HL = remaining
                LD	B, H
                LD	C, L            ; BC = remaining
                JR	.size_ok
.use_frame_size
                ; BC = FRAME_SIZE already
.size_ok
                EX	DE, HL          ; restore DE = (remaining - sent) or updated
                POP	DE              ; DE = original remaining
                POP	HL              ; HL = buffer pointer

                ; BC = frame payload size
                ; save remaining count
                PUSH	DE
                PUSH	HL
                PUSH	BC

                ; send this frame
                LD	A, (tx_seq)
                CALL	send_frame

                POP	BC              ; frame size
                POP	HL
                POP	DE

                ; advance buffer pointer
                ADD	HL, BC

                ; subtract from remaining
                EX	DE, HL
                OR	A
                SBC	HL, BC
                EX	DE, HL

                ; increment seq
                LD	A, (tx_seq)
                INC	A
                LD	(tx_seq), A

                JR	.frame_loop

; ============================================================================
; send_frame — send one frame on the wire
; A = seq, HL = payload pointer, BC = payload length
; Frame: SOF SEQ LEN_H LEN_L PAYLOAD CRC_H CRC_L
; ============================================================================
send_frame
                LD	(tx_frame_seq), A
                LD	(tx_frame_ptr), HL
                LD	(tx_frame_len), BC

                ; clear tx failure flag
                XOR	A
                LD	(tx_fail), A

                ; init CRC
                LD	HL, 0xFFFF
                LD	(crc_val), HL

                ; send SOF
                LD	A, SOF
                CALL	uart_tx

                ; send SEQ + update CRC
                LD	A, (tx_frame_seq)
                CALL	uart_tx
                LD	A, (tx_frame_seq)
                CALL	crc_update_a

                ; send LEN_H + update CRC
                LD	A, (tx_frame_len + 1)
                CALL	uart_tx
                LD	A, (tx_frame_len + 1)
                CALL	crc_update_a

                ; send LEN_L + update CRC
                LD	A, (tx_frame_len)
                CALL	uart_tx
                LD	A, (tx_frame_len)
                CALL	crc_update_a

                ; send payload + update CRC
                LD	BC, (tx_frame_len)
                LD	A, B
                OR	C
                JR	Z, .sf_crc      ; zero-length — skip payload

                LD	HL, (tx_frame_ptr)
.sf_payload
                LD	A, (HL)
                CALL	uart_tx
                ; bail out if uart_tx timed out (PC disconnected)
                LD	A, (tx_fail)
                OR	A
                JR	NZ, .sf_crc     ; skip rest, CRC will be wrong but don't care
                LD	A, (HL)         ; reload (uart_tx preserves AF but crc trashes A)
                PUSH	HL
                PUSH	BC
                CALL	crc_update_a
                POP	BC
                POP	HL
                INC	HL
                DEC	BC
                LD	A, B
                OR	C
                JR	NZ, .sf_payload

.sf_crc
                ; send CRC high then low
                LD	A, (crc_val + 1)
                CALL	uart_tx
                LD	A, (crc_val)
                CALL	uart_tx
                RET

tx_frame_seq    DB	0
tx_frame_ptr    DW	0
tx_frame_len    DW	0

; ============================================================================
; read_from_disk — read up to FLUSH_SIZE bytes from open file into RXBUF
; Returns: BC = bytes read
;          carry set if EOF was reached (last read returned non-zero)
; ============================================================================
read_from_disk
                LD	HL, RXBUF
                LD	DE, 0           ; DE = total bytes read
                LD	A, FLUSH_SIZE / 128
                LD	(rd_records), A ; records to read

.rd_loop
                ; set DMA
                PUSH	DE
                PUSH	HL
                LD	D, H
                LD	E, L
                LD	C, F_SETDMA
                CALL	BDOS

                ; read one 128-byte record
                LD	DE, FCB
                LD	C, F_READ
                CALL	BDOS
                POP	HL              ; restore buf ptr
                POP	DE              ; restore total

                OR	A
                JR	NZ, .rd_eof     ; BDOS returns non-zero at EOF

                ; advance
                LD	BC, 128
                ADD	HL, BC
                EX	DE, HL
                ADD	HL, BC
                EX	DE, HL

                LD	A, (rd_records)
                DEC	A
                LD	(rd_records), A
                JR	NZ, .rd_loop

                ; full chunk read, no EOF
                LD	B, D
                LD	C, E            ; BC = total bytes read
                OR	A               ; clear carry
                RET

.rd_eof
                ; EOF reached, return what we have
                LD	B, D
                LD	C, E
                SCF
                RET

rd_records      DB	0

; ============================================================================
; build_header_payload — construct "FILENAME.EXT\0" + 4-byte LE size in RXBUF
; Returns BC = payload length
; ============================================================================
build_header_payload
                LD	DE, RXBUF
                ; copy FCB name (8 bytes, strip trailing spaces)
                LD	HL, FCB + 1
                LD	B, 8
.bh_name
                LD	A, (HL)
                CP	' '
                JR	Z, .bh_name_done
                LD	(DE), A
                INC	HL
                INC	DE
                DJNZ	.bh_name

                JR	.bh_dot
.bh_name_done
                ; skip remaining spaces in name
.bh_dot
                ; check if extension has non-space chars
                LD	HL, FCB + 9
                LD	A, (HL)
                CP	' '
                JR	Z, .bh_no_ext

                ; add '.'
                LD	A, '.'
                LD	(DE), A
                INC	DE

                ; copy extension (up to 3 chars, strip trailing spaces)
                LD	B, 3
.bh_ext
                LD	A, (HL)
                CP	' '
                JR	Z, .bh_no_ext
                LD	(DE), A
                INC	HL
                INC	DE
                DJNZ	.bh_ext

.bh_no_ext
                ; null terminator
                XOR	A
                LD	(DE), A
                INC	DE

                ; 4-byte LE file size
                LD	HL, file_size
                LD	BC, 4
                LDIR

                ; BC = payload length = DE - RXBUF
                LD	HL, RXBUF
                EX	DE, HL
                OR	A
                SBC	HL, DE
                LD	B, H
                LD	C, L
                RET

; ============================================================================
; Print FCB filename (for status messages). Assembles "NAME.EXT\r\n$" into
; fname_buf and routes through print_user_msg so it follows the same
; wire-safe path (uart_tx for TTY users, BIOS for local-CRT users).
; ============================================================================
print_fcb_name
                LD	DE, fname_buf

                ; copy name (up to 8 chars, stop at first trailing space)
                LD	HL, FCB + 1
                LD	B, 8
.pn_name
                LD	A, (HL)
                CP	' '
                JR	Z, .pn_dot
                LD	(DE), A
                INC	HL
                INC	DE
                DJNZ	.pn_name

.pn_dot
                ; check extension
                LD	HL, FCB + 9
                LD	A, (HL)
                CP	' '
                JR	Z, .pn_done

                ; emit '.'
                LD	A, '.'
                LD	(DE), A
                INC	DE

                ; copy extension (up to 3 chars, stop at first trailing space)
                LD	B, 3
.pn_ext
                LD	A, (HL)
                CP	' '
                JR	Z, .pn_done
                LD	(DE), A
                INC	HL
                INC	DE
                DJNZ	.pn_ext

.pn_done
                ; append CRLF + '$' terminator
                LD	A, 13
                LD	(DE), A
                INC	DE
                LD	A, 10
                LD	(DE), A
                INC	DE
                LD	A, '$'
                LD	(DE), A

                LD	DE, fname_buf
                CALL	print_user_msg
                RET

fname_buf       DS	16

; ============================================================================
; CRC-16-CCITT routines
; ============================================================================

; Update CRC with byte in A
; Trashes: A, HL, DE.  Preserves BC.
crc_update_a
                PUSH	BC
                LD	B, A

                LD	A, (crc_val + 1)
                XOR	B
                LD	L, A
                LD	H, 0

                ADD	HL, HL
                LD	DE, CRC_TABLE
                ADD	HL, DE

                LD	A, (crc_val)
                XOR	(HL)
                LD	(crc_val + 1), A
                INC	HL
                LD	A, (HL)
                LD	(crc_val), A

                POP	BC
                RET

; Build CRC-16-CCITT lookup table at CRC_TABLE (512 bytes)
init_crc_table
                LD	HL, CRC_TABLE
                LD	C, 0

.table_loop
                LD	A, C
                LD	D, A
                LD	E, 0

                LD	B, 8
.bit_loop
                LD	A, D
                AND	0x80
                JR	Z, .no_xor

                SLA	E
                RL	D
                LD	A, D
                XOR	0x10
                LD	D, A
                LD	A, E
                XOR	0x21
                LD	E, A
                JR	.next_bit

.no_xor
                SLA	E
                RL	D

.next_bit
                DJNZ	.bit_loop

                LD	(HL), D
                INC	HL
                LD	(HL), E
                INC	HL

                INC	C
                JR	NZ, .table_loop

                RET

; ============================================================================
; Print A as two hex digits
; ============================================================================
print_hex_a
                PUSH    AF
        .4	RRCA
                AND     0xF
                CALL    .nibble
                POP	AF
                AND	0x0F
                CALL    .nibble
                RET
.nibble
                CP      10
                JR	C, .digit
                ADD	A, 'A' - 10
                JR      .out
.digit
                ADD	A, '0'
.out
                LD	E, A
                LD	C, C_WRITE
                CALL    BDOS
                RET

; ============================================================================
; print_user_msg - emit a $-terminated string to the user's console.
; DE = pointer to $-terminated string.
;
; For PC-driven users (original console = TTY, i.e. the serial UART), the
; message is sent directly via uart_tx. We must NOT route via BDOS/BIOS in
; this case: the MicroBeast BIOS's console-out path leaves the UART in a
; state that breaks subsequent flow control on this machine. Symptom: after
; the first BIOS-routed user message, Z80's RTS is deasserted (or AFE state
; is otherwise corrupted), PC's next write blocks on CTS, and both sides
; deadlock.
;
; For local-CRT users (original console != TTY) we still go through BDOS
; with the original IOBYTE so the message reaches their physical console.
; ============================================================================
print_user_msg
                ; CON=TTY ⇒ wire-direct via uart_tx (no BIOS, has 330ms timeout)
                LD	A, (iobyte_saved)
                AND	0x03            ; CON field (bits 0-1)
                JR	NZ, .via_bios

                PUSH	DE
.uart_loop
                LD	A, (DE)
                CP	'$'
                JR	Z, .uart_done
                CALL	uart_tx
                INC	DE
                JR	.uart_loop
.uart_done
                POP	DE
                RET

.via_bios
                ; Local-console user: route through BDOS with the user's
                ; original IOBYTE so it reaches their physical CON.
                LD	HL, IOBYTE
                LD	A, (HL)
                PUSH	AF              ; save current (redirect) value
                LD	A, (iobyte_saved)
                LD	(HL), A

                LD	C, C_WRITESTR
                CALL	BDOS

                POP	AF
                LD	HL, IOBYTE
                LD	(HL), A
                RET

; ============================================================================
; Messages
; ============================================================================
msg_banner_recv DB	"SLIDE v0.6.0-dev - Receive mode", 13, 10, '$'
msg_banner_send DB	"SLIDE v0.6.0-dev - Send mode", 13, 10, '$'
msg_sending     DB	"Sending: ", '$'
msg_done        DB	13, 10, "Transfer complete!", 13, 10, '$'
msg_done_session DB	13, 10, "Session complete.", 13, 10, '$'
msg_err_hdr     DB	13, 10, "Error: bad header frame", 13, 10, '$'
msg_err_handshake DB	13, 10, "Error: PC did not send RDY (handshake timeout)", 13, 10, '$'
msg_err_file    DB	13, 10, "Error: can't create file", 13, 10, '$'
msg_err_disk    DB	13, 10, "Error: disk write failed", 13, 10, '$'
msg_err_open    DB	13, 10, "Error: can't open file", 13, 10, '$'
msg_err_nopc    DB	13, 10, "Error: PC not responding", 13, 10, '$'
msg_dbg_ok      DB	13, 10, "DBG: header frame OK", 13, 10, '$'
msg_dbg_fail    DB	13, 10, "DBG: recv_frame failed", 13, 10, '$'
msg_dbg_crc     DB	"DBG: CRC mismatch cmp/prs: ", '$'
msg_dbg_tmo     DB	"DBG: timeout in payload", 13, 10, '$'
msg_err_abort   DB	13, 10, "Transfer aborted - connection lost", 13, 10, '$'
msg_cancelled   DB	13, 10, "Transfer cancelled by peer", 13, 10, '$'
                END	entry
