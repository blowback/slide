; ============================================================================
; SLIDE - Serial Line Inter-Device (file) Exchange
; Custom file transfer protocol for Z80 / CP/M
; Target: 8MHz Z80, TL16C550 UART with 16-byte FIFO, auto RTS/CTS flow control
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
CTRL_RDY        EQU	0x11             ; ready after disk flush
CTRL_EOT        EQU	0x04             ; end of transfer (zero-length frame)
CTRL_CAN        EQU	0x18             ; cancel (disk error)

WIN_SIZE        EQU	4                ; sliding window size
FRAME_SIZE      EQU	1024             ; payload bytes per frame
FLUSH_SIZE      EQU	WIN_SIZE * FRAME_SIZE ; 4KB - flush to disk threshold

; --- CP/M BDOS --------------------------------------------------------------
IOBYTE          EQU	0x0003
BDOS            EQU	0x0005
FCB             EQU	0x005C
DMA_ADDR        EQU	0x0080
F_OPEN          EQU	15
F_CLOSE         EQU	16
F_CREATE        EQU	22
F_DELETE        EQU	19
F_WRITE         EQU	21
F_SETDMA        EQU	26
C_WRITESTR      EQU	9
C_WRITE         EQU	2

; --- Memory layout -----------------------------------------------------------
; Adjust RXBUF to somewhere safe in TPA
RXBUF           EQU	0x8000           ; 4KB receive buffer (WIN_SIZE * FRAME_SIZE)
RXBUF_END       EQU	RXBUF + FLUSH_SIZE
CRC_TABLE       EQU	0x9000           ; 512 bytes for CRC-16-CCITT lookup table

; ============================================================================
; Entry point
; ============================================================================
                ORG	0x0100           ; CP/M TPA

entry
                ; stop the BIOS from using the UART
                LD	HL, IOBYTE      ; save current iobyte
                LD	A, (HL)
                LD	DE, iobyte_saved
                LD	(DE), A

                LD	A, 0b01010110   ; CON=BAT, RDR=PTR, LST=CRT
                LD	(HL), A

                CALL	init_crc_table
                CALL	uart_init

                ; print banner
                LD	DE, msg_banner
                LD	C, C_WRITESTR
                CALL	BDOS

                ; send RDY and wait for header, retrying until PC connects
                LD	E, 15            ; ~30 seconds (15 x ~2s timeout)
.wait_pc
                CALL	send_rdy
                CALL	recv_header
                JR	NC, .got_header
                ; timeout — PC not connected yet, try again
                CALL	uart_flush_rx
                DEC	E
                JR	NZ, .wait_pc
                ; gave up waiting
                LD	DE, msg_err_hdr
                LD	C, C_WRITESTR
                CALL	BDOS
                JR	.out
.got_header

                ; create output file
                CALL	create_file
                JR	C, .err_file

                ; send ACK for header
                LD	A, 0              ; ACK seq 0
                CALL	send_ack

                ; main receive loop
                CALL	recv_file

                ; close file (always, even on abort)
                CALL	close_file

                JR	.out

.err_header
                LD	DE, msg_err_hdr
                LD	C, C_WRITESTR
                CALL	BDOS
                CALL	send_can
                JR	.out

.err_file
                LD	DE, msg_err_file
                LD	C, C_WRITESTR
                CALL	BDOS
                CALL	send_can
                JR	.out

.out            ; restore original IOBYTE
                LD	HL, iobyte_saved
                LD	DE, IOBYTE
                LD	A, (HL)
                LD	(DE), A
                
                RST	0               ; warm start back to cp/m

iobyte_saved    db      0

; ============================================================================
; UART initialisation
; Set up 16C650: 19200 baud, 8N1, FIFOs enabled, RTS/CTS flow control
; Adjust divisor for crystal/clock
; ============================================================================
uart_init
                ; enable DLAB to set baud rate
                LD	A, 0x83           ; DLAB=1, 8 data bits, 1 stop, no parity
                OUT	(UART_LCR), A

                ; divisor for 19200
                ; e.g. 1.8432MHz crystal: divisor = 6
                LD	A, 6              ; divisor low byte
                OUT	(UART_BASE + 0), A
                XOR	A                ; divisor high byte
                OUT	(UART_BASE + 1), A

                ; clear DLAB, keep 8N1
                LD	A, 0x03
                OUT	(UART_LCR), A

                ; enable FIFOs, 8 byte trigger level, clear both
                LD	A, 0x87           ; enable FIFO, clear RX+TX, trigger=8
                OUT	(UART_FCR), A

                ; enable RTS, enable auto flow control
                LD	A, MCR_RTS | MCR_AFE  ; RTS=1, AFE=1 (auto flow control)
                OUT	(UART_MCR), A

                ; disable interrupts (we poll)
                XOR	A
                OUT	(UART_IER), A

                RET

; ============================================================================
; Receive a single byte from UART
; Returns: A = byte received
; Blocks until data available
; ============================================================================
uart_rx
                IN	A, (UART_LSR)
                BIT	0, A             ; LSR_DR
                JR	Z, uart_rx
                IN	A, (UART_RBR)
                RET

; ============================================================================
; Receive byte with timeout
; Returns: A = byte, carry clear on success
;          carry set on timeout
; Trashes: B (outer), D (inner counter)
; ============================================================================
uart_rx_timeout
                LD	B, 0              ; outer loop ~2 seconds at 8MHz
.outer
                LD	D, 0              ; inner loop
.inner
                IN	A, (UART_LSR)
                BIT	0, A
                JR	NZ, .got_byte
                DEC	D
                JR	NZ, .inner
                DEC	B
                JR	NZ, .outer
                SCF                  ; timeout - set carry
                RET
.got_byte
                IN	A, (UART_RBR)
                OR	A                 ; clear carry
                RET

; ============================================================================
; Send a single byte via UART
; A = byte to send
; ============================================================================
uart_tx
                PUSH	AF
                PUSH	BC
                LD	B, 0              ; timeout counter (~330ms, same as rx)
                LD	C, 0
.wait
                IN	A, (UART_LSR)
                BIT	5, A             ; LSR_THRE
                JR	NZ, .ready
                DEC	C
                JR	NZ, .wait
                DEC	B
                JR	NZ, .wait
                POP	BC                ; timeout - discard byte
                POP	AF
                RET
.ready
                POP	BC
                POP	AF
                OUT	(UART_THR), A
                RET

; ============================================================================
; Send ACK [seq]
; A = sequence number to acknowledge
; ============================================================================
send_ack
                PUSH	AF
                LD	A, CTRL_ACK
                CALL	uart_tx
                POP	AF
                CALL	uart_tx
                RET

; ============================================================================
; Send NAK [seq]
; A = sequence number to request retransmit from
; ============================================================================
send_nak
                PUSH	AF
                LD	A, CTRL_NAK
                CALL	uart_tx
                POP	AF
                CALL	uart_tx
                RET

; ============================================================================
; Send RDY
; ============================================================================
send_rdy
                LD	A, CTRL_RDY
                CALL	uart_tx
                RET

; ============================================================================
; Flush UART receive FIFO (drain any stale bytes)
; ============================================================================
uart_flush_rx
.flush_loop
                IN	A, (UART_LSR)
                BIT	0, A             ; data ready?
                RET	Z                ; no more data
                IN	A, (UART_RBR)    ; read and discard
                JR	.flush_loop

; ============================================================================
; Send CAN (cancel - disk error)
; ============================================================================
send_can
                LD	A, CTRL_CAN
                CALL	uart_tx
                RET

; ============================================================================
; Receive frame header, validate, receive payload + CRC
;
; Expects: SOF SEQ LEN_H LEN_L [PAYLOAD] CRC_H CRC_L
;
; On entry: HL = destination buffer for payload
; Returns:  carry clear = success
;               A = sequence number
;               BC = payload length (0 = end of transfer)
;           carry set = CRC error or timeout
; ============================================================================
recv_frame
                ; wait for SOF
.wait_sof
                CALL	uart_rx_timeout
                JP	C, .fail_sof
                CP	SOF
                JR	NZ, .wait_sof

                ; --- begin CRC over SEQ+LEN+PAYLOAD ---
                LD	(frame_dst), HL   ; save dest ptr
                LD	HL, 0xFFFF        ; CRC init value
                LD	(crc_val), HL

                ; receive SEQ
                CALL	uart_rx_timeout
                JP	C, .fail_seq
                LD	(rx_seq), A
                CALL	crc_update_a

                ; receive LEN_H
                CALL	uart_rx_timeout
                JP	C, .fail_lenh
                LD	(rx_len + 1), A     ; high byte
                CALL	crc_update_a

                ; receive LEN_L
                CALL	uart_rx_timeout
                JP	C, .fail_lenl
                LD	(rx_len), A       ; low byte
                CALL	crc_update_a

                ; check for zero-length (end of transfer)
                LD	BC, (rx_len)
                LD	A, B
                OR	C
                JR	Z, .recv_crc      ; no payload, just get CRC

                ; receive payload bytes into (frame_dst), length in BC
                LD	HL, (frame_dst) ; restore dest ptr
                PUSH	HL              ; save buffer start
                PUSH	BC              ; save length
.recv_payload
                PUSH	BC              ; save payload counter (uart_rx_timeout trashes B)
                CALL	uart_rx_timeout
                POP	BC              ; restore payload counter
                JR	C, .payload_err
                LD	(HL), A
                INC	HL
                LD	(frame_dst), HL ; save updated ptr before CRC trashes HL
                CALL	crc_update_a
                LD	HL, (frame_dst) ; restore ptr

                DEC	BC
                LD	A, B
                OR	C
                JR	NZ, .recv_payload
                POP	BC               ; restore length
                POP	HL               ; restore buffer start
                JR	.recv_crc

.payload_err
                POP	BC
                POP	HL
                ; debug: print timeout indicator
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
                LD	(rx_crc + 1), A     ; expected CRC high

                CALL	uart_rx_timeout
                JR	C, .fail_crcl
                LD	(rx_crc), A       ; expected CRC low

                ; compare computed CRC with received CRC
                LD	HL, (crc_val)
                LD	DE, (rx_crc)
                OR	A                 ; clear carry
                SBC	HL, DE
                JR	NZ, .crc_err

                ; success!
                LD	A, (rx_seq)
                LD	BC, (rx_len)
                OR	A                 ; clear carry
                RET

.crc_err
                ; debug: print computed vs presentation CRCs
                PUSH	HL
                LD	DE, msg_dbg_crc
                LD	C, C_WRITESTR
                CALL    BDOS

                ; print computed CRC
                LD	HL, (crc_val)
                LD	A, H
                CALL    print_hex_a
                LD	A, L
                CALL    print_hex_a

                ; space
                LD	E, ' '
                LD	C, C_WRITE
                CALL    BDOS

                ; print presentation CRC
                LD	HL, (rx_crc)
                LD	A, H
                CALL    print_hex_a
                LD	A, L
                CALL    print_hex_a

                ; CRLF
                LD	E, 13
                LD	C, C_WRITE
                CALL    BDOS
                LD	E, 10
                LD	C, C_WRITE
                CALL    BDOS

                POP     HL

                SCF
                RET

; --- debug: print char in A then set carry and return ---
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

; --- frame receive temporaries ---
rx_seq          DB	0
rx_len          DW	0
rx_crc          DW	0
crc_val         DW	0
crc_accum       DW	0
frame_dst       DW	0

; ============================================================================
; Receive header frame
; Header payload: null-terminated filename, then 4 bytes file size (little-endian)
; Returns: carry clear = success, carry set = error
; ============================================================================
recv_header
                LD	HL, RXBUF         ; use start of buffer temporarily
                CALL	recv_frame
                JR	C, .hdr_fail

                ; print received seq and length for debug
                LD	DE, msg_dbg_ok
                LD	C, C_WRITESTR
                CALL	BDOS

                ; copy filename from RXBUF into FCB
                CALL	parse_filename
                RET

.hdr_fail
                ; debug: print what went wrong
                ; show first few bytes from UART to diagnose
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
                JR	Z, .pad_name      ; end of string, no extension
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
                ; pad remaining name chars with spaces
                ; calculate how many slots remain before extension
                PUSH	HL
                LD	HL, FCB + 9
                OR	A
                SBC	HL, DE
                LD	B, L              ; bytes to pad
                POP	HL
                LD	A, B
                OR	A
                JR	Z, .ext_start     ; no padding needed (name was 8 chars)
.pad_n
                LD	A, ' '
                LD	(DE), A
                INC	DE
                DJNZ	.pad_n

.ext_start
                INC	HL               ; skip '.'
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
                ; find the null
                LD	HL, RXBUF
.find_null
                LD	A, (HL)
                INC	HL
                OR	A
                JR	NZ, .find_null
                ; HL now points to file size bytes
                LD	DE, file_size
                LD	BC, 4
                LDIR

                OR	A                 ; clear carry
                RET

file_size       DW	0, 0             ; 32-bit file size

; ============================================================================
; Create output file via CP/M BDOS
; Returns: carry set on error
; ============================================================================
create_file
                ; delete existing file (ignore error)
                LD	DE, FCB
                LD	C, F_DELETE
                CALL	BDOS

                ; create new file
                LD	DE, FCB
                LD	C, F_CREATE
                CALL	BDOS
                CP	0xFF
                JR	Z, .create_err
                OR	A                 ; clear carry
                RET
.create_err
                SCF
                RET

; ============================================================================
; Close file
; ============================================================================
close_file
                LD	DE, FCB
                LD	C, F_CLOSE
                CALL	BDOS
                RET

; ============================================================================
; Main file receive loop
; Receives frames with sliding window, buffers in RAM, flushes to disk
; ============================================================================
recv_file
                LD	A, 1            ; first data frame is seq 1 (0 was header)
                LD	(expected_seq), A
                LD	HL, RXBUF
                LD	(buf_ptr), HL
                LD	HL, 0
                LD	(buf_used), HL

.recv_loop
                ; calculate buffer position for next frame
                LD	HL, (buf_ptr)
                CALL	recv_frame
                JR	C, .handle_error

                ; save seq before zero-length check clobbers A
                LD	D, A              ; D = received seq

                ; frame received - reset retry counter
                XOR	A
                LD	(retry_count), A

                ; check for end of transfer (zero-length frame)
                LD	A, B
                OR	C
                JR	Z, .end_of_file

                ; verify sequence number
                LD	A, (expected_seq)
                CP	D
                JR	NZ, .seq_error

                ; frame good - advance buffer pointer
                LD	HL, (buf_ptr)
                ADD	HL, BC           ; advance by payload length
                LD	(buf_ptr), HL

                ; track total buffered
                LD	HL, (buf_used)
                ADD	HL, BC
                LD	(buf_used), HL

                ; increment expected sequence
                LD	A, (expected_seq)
                INC	A
                LD	(expected_seq), A

                ; flush before ACK so PC doesn't send during disk I/O
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
                DEC	A                ; A = last received seq
                AND	WIN_SIZE - 1     ; 0 when seq is multiple of WIN_SIZE
                JR	NZ, .recv_loop

                ; send ACK (after any flush is complete)
                LD	A, (expected_seq)
                DEC	A
                CALL	send_ack
                JR	.recv_loop

.handle_error
                ; CRC error or timeout - check retry limit
                LD	A, (retry_count)
                INC	A
                LD	(retry_count), A
                CP	MAX_RETRIES
                JR	NC, .abort

                ; NAK the expected sequence
                LD	A, (expected_seq)
                CALL	send_nak
                JR	.recv_loop

.seq_error
                ; out of sequence - NAK what we expected
                LD	A, (expected_seq)
                CALL	send_nak
                JR	.recv_loop

.abort
                ; too many retries - give up
                LD	DE, msg_err_abort
                LD	C, C_WRITESTR
                CALL	BDOS
                RET

.disk_error
                ; send CAN to tell PC to stop, then bail out
                CALL	send_can
                RET

.end_of_file
                ; flush any remaining data in buffer
                LD	HL, (buf_used)
                LD	A, H
                OR	L
                JR	Z, .eof_ack
                CALL	flush_to_disk
                JR	C, .disk_error

.eof_ack
                ; ACK the final frame
                LD	A, (expected_seq)
                CALL	send_ack

                LD	DE, msg_done
                LD	C, C_WRITESTR
                CALL	BDOS
                RET

; --- recv state ---
expected_seq    DB	0
buf_ptr         DW	RXBUF
buf_used        DW	0
retry_count     DB	0

; ============================================================================
; Flush buffer to disk via CP/M sequential writes
; Writes (buf_used) bytes from RXBUF in 128-byte records
; ============================================================================
flush_to_disk
                LD	HL, RXBUF
                LD	DE, (buf_used)

.write_loop
                ; any data left?
                LD	A, D
                OR	E
                RET	Z

                ; set DMA address to current position in buffer
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
                LD	A, (FCB + 32)    ; save current record before write
                PUSH	AF
                LD	DE, FCB
                LD	C, F_WRITE
                CALL	BDOS
                POP	BC               ; B = old current record
                POP	DE
                POP	HL

                ; check for write error (BDOS return or record didn't advance)
                OR	A
                JR	NZ, .write_err
                LD	A, (FCB + 32)
                CP	B
                JR	Z, .write_err    ; record didn't advance = silent failure

                ; advance 128 bytes
                LD	BC, 128
                ADD	HL, BC

                ; subtract 128 from remaining count
                EX	DE, HL
                OR	A
                SBC	HL, BC
                ; if we went negative (partial last record), we're done
                JR	C, .write_done
                EX	DE, HL
                JR	.write_loop

.write_done
                OR	A                ; clear carry (success)
                RET

.write_err
                LD	DE, msg_err_disk
                LD	C, C_WRITESTR
                CALL	BDOS
                SCF                  ; set carry (error)
                RET

; ============================================================================
; CRC-16-CCITT routines
; Polynomial: 0x1021, init: 0xFFFF
; ============================================================================

; Update CRC with byte in A
; Uses 512-byte lookup table for speed
; Modifies: crc_val
; Trashes: A, HL, DE
crc_update_a
                PUSH	BC
                LD	B, A              ; save input byte

                ; index = (crc_high XOR input_byte)
                LD	A, (crc_val + 1)  ; CRC high byte
                XOR	B                ; XOR with input byte
                LD	L, A
                LD	H, 0

                ; table entry = CRC_TABLE + index * 2
                ADD	HL, HL           ; * 2
                LD	DE, CRC_TABLE
                ADD	HL, DE

                ; new_crc_high = table_high XOR crc_low
                ; new_crc_low = table_low
                LD	A, (crc_val)      ; CRC low byte
                XOR	(HL)             ; XOR with table low byte
                LD	(crc_val + 1), A  ; becomes new high byte
                INC	HL
                LD	A, (HL)           ; table high byte
                LD	(crc_val), A      ; becomes new low byte

                POP	BC
                RET

; ============================================================================
; Build CRC-16-CCITT lookup table at CRC_TABLE (512 bytes)
; ============================================================================
init_crc_table
                LD	HL, CRC_TABLE
                LD	C, 0              ; byte index 0..255

.table_loop
                ; CRC = index << 8
                LD	A, C
                LD	D, A              ; D = high byte
                LD	E, 0              ; E = low byte

                ; process 8 bits
                LD	B, 8
.bit_loop
                ; test high bit of D
                LD	A, D
                AND	0x80
                JR	Z, .no_xor

                ; shift DE left
                SLA	E
                RL	D
                ; XOR with polynomial 0x1021
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

                ; store in table: high byte first, then low
                LD	(HL), D
                INC	HL
                LD	(HL), E
                INC	HL

                INC	C
                JR	NZ, .table_loop   ; loop 256 times

                RET

; ============================================================================
; Print A as two hex digits to console
; ============================================================================
print_hex_a
                PUSH    AF
                ; high nibble
        .4	RRCA
                AND     0xf        
                CALL    .nibble
                POP	AF
                ; low nibble
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
msg_banner      DB	"SLIDE v0.1 - Waiting for transfer...", 13, 10, '$'
msg_done        DB	13, 10, "Transfer complete!", 13, 10, '$'
msg_err_hdr     DB	13, 10, "Error: bad header frame", 13, 10, '$'
msg_err_file    DB	13, 10, "Error : can't create file", 13, 10, '$'
msg_err_disk    DB	13, 10, "Error: disk write failed", 13, 10, '$'
msg_dbg_ok      DB	13, 10, "DBG: header frame OK", 13, 10, '$'
msg_dbg_fail    DB	13, 10, "DBG: recv_frame failed", 13, 10, '$'
msg_dbg_crc     DB	"DBG: CRC mismatch cmp/prs: ", '$'
msg_dbg_tmo     DB	"DBG: timeout in payload", 13, 10, '$'
msg_err_abort   DB	13, 10, "Transfer aborted - connection lost", 13, 10, '$'
                END	entry
