; ============================================================================
; SLIDE - Serial Line Inter-Device (file) Exchange
; Custom file transfer protocol for Z80 / CP/M
; Target: 8MHz Z80, TL16C550 UART with 16-byte FIFO, auto RTS/CTS flow control
; ============================================================================
; When debugging slide, it's crucially important to have nobody else using
; the UART, on both the microbeast end and the PC end.
; On the PC end this is easy: just remember to kill minicom after ymodem
; On the BEAST end, slide.com locks out the BIOS from the UART, but the
; beast is weirdly sensitive to the lack of a controlling terminal when you
; are typing on the keyboard. The solution is to (from minicom) type:
;
;  stat RDR:=PTR:
;  stat LST:=CRT:
;  stat CON:=BAT:
;
; now the BEAST is entirely in its own screen/keyboard and slide is safe to
; use the serial port.
;
; typing that is a bit of a faff every time, hence this utility!

; --- CP/M BDOS --------------------------------------------------------------
IOBYTE          EQU	0x0003
BDOS            EQU	0x0005
C_WRITESTR      EQU	9

; ============================================================================
; Entry point
; ============================================================================
		OUTPUT	reset.com
                ORG	0x0100           ; CP/M TPA

entry
		LD	DE, banner
		LD	C, C_WRITESTR
		CALL	BDOS

                ; stop the BIOS from using the UART
                LD	A, 0b01010110   ; CON=BAT, RDR=PTR, LST=CRT
                LD	HL, IOBYTE    
                LD	(HL), A

		RET
		; RST	0

banner		DB	13, 10, "DISCONNECT MINICOM!!!", 13, 10,'$'

		END
