;********************************************************************************
; led_blink.asm: Blinkning a LED connected to pin 8 (PORTB0) by pressdown of
;                a button connected to pin 13 (PORTB5).
;********************************************************************************

; Macro definitions:
.EQU LED1       = PORTB0 ; LED 1 connected to pin 8 (PORTB0).
.EQU BUTTON1    = PORTB5 ; BUTTON1 connected to pin 13 (PORTB5).
.EQU RESET_vect = 0x00   ; Reset vector, program start address.

;********************************************************************************
; .CSEG: Code segment (program memory), storage space for machine code.
;********************************************************************************
.CSEG

;********************************************************************************
; RESET_vect: Calling the main subroutine to start the program.
;********************************************************************************
.ORG RESET_vect
   RJMP main 

;********************************************************************************
; main: Initiates the I/O ports at start. The program is running as long as
;       voltage is supplied. BUTTON1 is continuously polled. When BUTTON1 
;       is pressed, LED1 is enabled, otherwise it's disabled.
;********************************************************************************
main:
   CALL setup
main_loop:
   IN R16, PINB
   ANDI R16, (1 << BUTTON1)
   BRNE led1_on

;********************************************************************************
; led1_off: Disables LED1.
;********************************************************************************
led1_off:
   IN R16, PORTB
   ANDI R16, ~(1 << LED1)
   OUT PORTB, R16
   RJMP main_loop

;********************************************************************************
; led1_on: Enables LED1.
;********************************************************************************
led1_on:
   IN R16, PORTB
   ORI R16, (1 << LED1)
   OUT PORTB, R16
   RJMP main_loop

;********************************************************************************
; setup: Sets the pin connected to LED1 to output and enables the internal 
;        pull-up resistor for the pin connected to BUTTON1 so that the input
;        signal always reads as high (1) or low (0).
;********************************************************************************
setup:
   LDI R16, (1 << LED1)
   OUT DDRB, R16
   LDI R16, (1 << BUTTON1)
   OUT PORTB, R16
   RET

