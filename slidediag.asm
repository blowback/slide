; ============================================================================
; SLIDE UART diagnostic - echo test
; Receives bytes on UART and prints them as hex to the console
; Press any console key to exit
; ============================================================================

UART_BASE       EQU 0x20             ; change to match your board
UART_RBR        EQU UART_BASE + 0
UART_THR        EQU UART_BASE + 0
UART_IER        EQU UART_BASE + 1
UART_FCR        EQU UART_BASE + 2
UART_LCR        EQU UART_BASE + 3
UART_MCR        EQU UART_BASE + 4
UART_LSR        EQU UART_BASE + 5

MCR_RTS         EQU 0x02
MCR_AFE         EQU 0x20

BDOS            EQU 0x0005
C_WRITE         EQU 2
C_WRITESTR      EQU 9
C_STATUS        EQU 11

                ORG 0x0100
                OUTPUT slidiag.com

entry:
                CALL uart_init

                LD DE, msg_banner
                LD C, C_WRITESTR
                CALL BDOS

                ; main loop: check UART for data, print as hex
.loop:
                ; check if console key pressed (exit)
                LD C, C_STATUS
                CALL BDOS
                OR A
                JR NZ, .exit

                ; check UART for received byte
                IN A, (UART_LSR)
                BIT 0, A             ; data ready?
                JR Z, .loop

                ; read the byte
                IN A, (UART_RBR)

                ; also echo it back out the UART
                PUSH AF
                OUT (UART_THR), A
                POP AF

                ; print as hex to console
                CALL print_hex_a
                LD E, ' '
                LD C, C_WRITE
                CALL BDOS

                JR .loop

.exit:
                LD DE, msg_done
                LD C, C_WRITESTR
                CALL BDOS
                RST 0

; ============================================================================
; Print A as two hex digits to console
; ============================================================================
print_hex_a:
                PUSH AF
                ; high nibble
                RRCA
                RRCA
                RRCA
                RRCA
                AND 0x0F
                CALL .nibble
                POP AF
                ; low nibble
                AND 0x0F
                CALL .nibble
                RET

.nibble:
                CP 10
                JR C, .digit
                ADD A, 'A' - 10
                JR .print
.digit:
                ADD A, '0'
.print:
                LD E, A
                LD C, C_WRITE
                CALL BDOS
                RET

; ============================================================================
; UART init - same as SLIDE main
; ============================================================================
uart_init:
                LD A, 0x83
                OUT (UART_LCR), A
                LD A, 6
                OUT (UART_BASE + 0), A
                XOR A
                OUT (UART_BASE + 1), A
                LD A, 0x03
                OUT (UART_LCR), A
                LD A, 0x87
                OUT (UART_FCR), A
                LD A, MCR_RTS | MCR_AFE
                OUT (UART_MCR), A
                XOR A
                OUT (UART_IER), A
                RET

msg_banner:     DB "SLIDE UART diag - send bytes from PC, press key to exit", 13, 10, '$'
msg_done:       DB 13, 10, "Done.", 13, 10, '$'

                END entry
