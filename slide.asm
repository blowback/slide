; ============================================================================
; SLIDE v0.2 - Serial Line Inter-Device (file) Exchange
; Custom file transfer protocol for Z80 / CP/M
; Target: 8MHz Z80, TL16C550 UART with 16-byte FIFO, auto RTS/CTS flow control
;
; Usage:  SLIDE              — receive mode (default)
;         SLIDE R            — receive mode (explicit)
;         SLIDE S FILE.COM   — send FILE.COM to PC
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
                LD	C, C_WRITESTR
                CALL	BDOS
                CALL	recv_session
                JR	.out

.do_send
                ; --- send mode ---
                LD	DE, msg_banner_send
                LD	C, C_WRITESTR
                CALL	BDOS
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
                ; copy filename to send_fname (null-terminated, max 12 chars)
                LD	DE, send_fname
                LD	C, 12           ; max filename length
.copy_fname
                LD	A, (HL)
                CP	' '
                JR	Z, .fname_done
                OR	A
                JR	Z, .fname_done
                LD	(DE), A
                INC	HL
                INC	DE
                DEC	C
                JR	Z, .fname_done  ; truncate at 12
                DJNZ	.copy_fname
.fname_done
                XOR	A
                LD	(DE), A         ; null terminate
                RET

mode            DB	0               ; 0=receive, 1=send
send_fname      DS	13              ; null-terminated filename for send mode

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

; Send CAN (cancel)
send_can
                LD	A, CTRL_CAN
                CALL	uart_tx
                RET

; Send FIN
send_fin
                LD	A, CTRL_FIN
                CALL	uart_tx
                RET

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
                JR	Z, .no_seq
                ; stray byte (RDY etc) — keep waiting
                JR	.wait

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
                ; wait for SOF (or FIN)
.wait_sof
                CALL	uart_rx_timeout
                JP	C, .fail_sof
                CP	SOF
                JR	Z, .after_sof
                CP	CTRL_FIN
                JR	Z, .got_fin
                JR	.wait_sof

.got_fin
                ; signal FIN to caller via D=CTRL_FIN, carry set
                LD	D, CTRL_FIN
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
                ; --- Handshake: wait for PC's RDY, echo back ---
                LD	E, 15           ; ~30s (15 × ~2s timeout)
.wait_pc
                CALL	uart_rx_timeout
                JR	C, .wait_retry
                CP	CTRL_RDY
                JR	Z, .pc_ready
                ; not RDY — ignore and keep waiting
                JR	.wait_pc

.wait_retry
                DEC	E
                JR	NZ, .wait_pc
                ; gave up
                LD	DE, msg_err_hdr
                LD	C, C_WRITESTR
                CALL	BDOS
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
                CP	CTRL_FIN
                JR	Z, .got_fin
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

.got_fin
                ; echo FIN back
                CALL	send_fin

                LD	DE, msg_done_session
                LD	C, C_WRITESTR
                CALL	BDOS
                RET

.file_error
                ; recv_file already printed error and sent CAN — just exit
                RET

.err_header
                LD	DE, msg_err_hdr
                LD	C, C_WRITESTR
                CALL	BDOS
                CALL	send_can
                RET

.err_file
                LD	DE, msg_err_file
                LD	C, C_WRITESTR
                CALL	BDOS
                CALL	send_can
                RET

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
                JR	Z, .end_of_file

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
                LD	A, (retry_count)
                INC	A
                LD	(retry_count), A
                CP	MAX_RETRIES
                JR	NC, .abort

                LD	A, (expected_seq)
                CALL	send_nak
                JR	.recv_loop

.seq_error
                LD	A, (expected_seq)
                CALL	send_nak
                JR	.recv_loop

.abort
                LD	DE, msg_err_abort
                LD	C, C_WRITESTR
                CALL	BDOS
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
                LD	C, C_WRITESTR
                CALL	BDOS
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
                LD	C, C_WRITESTR
                CALL	BDOS
                SCF
                RET

; ============================================================================
; SEND SESSION (multi-file — currently single file from command line)
; Handshake: Z80 (sender) sends RDY, waits for PC's RDY echo.
; Then: open file → send_file_tx → close → send FIN, wait for FIN echo.
; ============================================================================
send_session
                ; --- Handshake: sender sends RDY, waits for PC's RDY echo ---
                LD	E, 15           ; ~30s
.wait_pc
                CALL	send_rdy
                CALL	uart_rx_timeout
                JR	C, .wait_retry
                CP	CTRL_RDY
                JR	Z, .pc_ready
                ; not RDY — keep trying
                JR	.wait_pc

.wait_retry
                DEC	E
                JR	NZ, .wait_pc
                ; gave up
                LD	DE, msg_err_nopc
                LD	C, C_WRITESTR
                CALL	BDOS
                RET

.pc_ready
                CALL	uart_flush_rx

                ; --- Set up FCB from send_fname ---
                ; Copy send_fname into RXBUF so parse_filename can use it
                LD	HL, send_fname
                LD	DE, RXBUF
                LD	BC, 13
                LDIR

                CALL	parse_filename

                ; print filename
                LD	DE, msg_sending
                LD	C, C_WRITESTR
                CALL	BDOS
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

                ; --- Send FIN, wait for echo ---
                CALL	send_fin
                ; wait for FIN echo (brief timeout)
                CALL	recv_control_z80
                ; don't care about result — we're done

                LD	DE, msg_done_session
                LD	C, C_WRITESTR
                CALL	BDOS
                RET

.err_open
                LD	DE, msg_err_open
                LD	C, C_WRITESTR
                CALL	BDOS
                RET

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
                LD	C, C_WRITESTR
                CALL	BDOS
                RET

.tx_done
                LD	DE, msg_done
                LD	C, C_WRITESTR
                CALL	BDOS
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
; Print FCB filename (for status messages)
; ============================================================================
print_fcb_name
                ; print name (8 chars, skip trailing spaces)
                LD	HL, FCB + 1
                LD	B, 8
.pn_name
                LD	A, (HL)
                CP	' '
                JR	Z, .pn_dot
                LD	E, A
                PUSH	HL
                PUSH	BC
                LD	C, C_WRITE
                CALL	BDOS
                POP	BC
                POP	HL
                INC	HL
                DJNZ	.pn_name

.pn_dot
                ; check extension
                LD	HL, FCB + 9
                LD	A, (HL)
                CP	' '
                JR	Z, .pn_done

                ; print '.'
                LD	E, '.'
                PUSH	HL
                LD	C, C_WRITE
                CALL	BDOS
                POP	HL

                ; print extension
                LD	B, 3
.pn_ext
                LD	A, (HL)
                CP	' '
                JR	Z, .pn_done
                LD	E, A
                PUSH	HL
                PUSH	BC
                LD	C, C_WRITE
                CALL	BDOS
                POP	BC
                POP	HL
                INC	HL
                DJNZ	.pn_ext

.pn_done
                ; CRLF
                LD	E, 13
                LD	C, C_WRITE
                CALL	BDOS
                LD	E, 10
                LD	C, C_WRITE
                CALL	BDOS
                RET

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
; Messages
; ============================================================================
msg_banner_recv DB	"SLIDE v0.2 - Receive mode", 13, 10, '$'
msg_banner_send DB	"SLIDE v0.2 - Send mode", 13, 10, '$'
msg_sending     DB	"Sending: ", '$'
msg_done        DB	13, 10, "Transfer complete!", 13, 10, '$'
msg_done_session DB	13, 10, "Session complete.", 13, 10, '$'
msg_err_hdr     DB	13, 10, "Error: bad header frame", 13, 10, '$'
msg_err_file    DB	13, 10, "Error: can't create file", 13, 10, '$'
msg_err_disk    DB	13, 10, "Error: disk write failed", 13, 10, '$'
msg_err_open    DB	13, 10, "Error: can't open file", 13, 10, '$'
msg_err_nopc    DB	13, 10, "Error: PC not responding", 13, 10, '$'
msg_dbg_ok      DB	13, 10, "DBG: header frame OK", 13, 10, '$'
msg_dbg_fail    DB	13, 10, "DBG: recv_frame failed", 13, 10, '$'
msg_dbg_crc     DB	"DBG: CRC mismatch cmp/prs: ", '$'
msg_dbg_tmo     DB	"DBG: timeout in payload", 13, 10, '$'
msg_err_abort   DB	13, 10, "Transfer aborted - connection lost", 13, 10, '$'
                END	entry
