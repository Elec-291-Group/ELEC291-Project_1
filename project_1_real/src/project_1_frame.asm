; Project 1 Frame Code
; CV-8052 microcontroller in DE10-Lite board


; org applied to current segment, default is CSEG
; ----------------------------------------------------------------------------------------------;
; Reset vector & Interrupt Vectors

; Reset vector
org 0x0000
    ljmp main

; External interrupt 0 vector
org 0x0003
	reti

; Timer/Counter 0 overflow interrupt vector
org 0x000B
	ljmp Timer0_ISR

; External interrupt 1 vector
org 0x0013
	reti

; Timer/Counter 1 overflow interrupt vector
org 0x001B
	reti

; Serial port receive/transmit interrupt vector
org 0x0023 
	reti
	
; Timer/Counter 2 overflow interrupt vector
org 0x002B
	ljmp Timer2_ISR
; ----------------------------------------------------------------------------------------------;

; ----------------------------------------------------------------------------------------------;
; include 
$NOLIST
$include(..\inc\MODMAX10)
$include(..\inc\LCD_4bit_DE10Lite.inc) ; LCD related functions and utility macros
$include(..\inc\math32.asm) ; 
$LIST
; ----------------------------------------------------------------------------------------------;

; ---------------------------------------------------------------------------------------------;
; Data Segment 0x30 -- 0x7F  (overall 79d bytes available)
dseg at 0x30
current_time_sec:     ds 1
current_time_minute:  ds 1
current_time_hour:    ds 1

; math32 buffer variables
x:		ds	4
y:		ds	4
bcd:	ds	5

current_temp: ds 4 ;
soak_temp:    ds 4 ;
reflow_temp:  ds 4 ;

current_time: ds 4 ;
soak_time:    ds 4 ;
reflow_time:  ds 4 ;

next_state:   ds 1 ;
current_state:ds 1 ;

power_output:  ds 4 ;

KEY1_DEB_timer: ds 1
SEC_FSM_timer: ds 1

KEY1_DEB_state: ds 1
SEC_FSM_state: ds 1
; 47d bytes used
; ---------------------------------------------------------------------------------------------;


; ---------------------------------------------------------------------------------------------;
; bit operation setb, clr, jb, and jnb
bseg
mf:		dbit 1 ; math32 sign
one_second_flag: dbit 1

soak_temp_reached: dbit 1
reflow_temp_reached: dbit 1
cooling_temp_reached: dbit 1

soak_time_reached: dbit 1
reflow_time_reached: dbit 1

reset_signal: dbit 1
stop_signal: dbit 1
start_signal: dbit 1

Key1_flag: dbit 1
; ---------------------------------------------------------------------------------------------;

; Code start here
; -----------------------------------------------------------------------------------------------------------------;
cseg
CLK            EQU 33333333 ; Microcontroller system crystal frequency in Hz
BAUD 		   EQU 57600

TIMER0_RATE    EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD  EQU ((65536-(CLK/(12*TIMER0_RATE)))) ; The prescaler in the CV-8052 is always 12 unlike the N76E003 where is selectable.

TIMER_1_RELOAD EQU (256-((2*CLK)/(12*32*BAUD)))

TIMER2_RATE    EQU 1000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD  EQU ((65536-(CLK/(12*TIMER2_RATE))))

SOUND_OUT      EQU P1.5 ; Pin connected to the speaker

; These 'equ' must match the wiring between the DE10Lite board and the LCD!
; P0 is in connector JPIO.  Check "CV-8052 Soft Processor in the DE10Lite Board: Getting
; Started Guide" for the details.
ELCD_RS equ P3.7
ELCD_RW equ P3.5
ELCD_E  equ P3.3
ELCD_D4 equ P3.1
ELCD_D5 equ P2.7
ELCD_D6 equ P2.5
ELCD_D7 equ P2.3

;                     1234567890123456    <- This helps determine the location of the counter
Initial_Message:  db 'initial message', 0

;----------------------------------------------------------------------------------------------;
; Timers Setting:
;   Timer 0: 2kHz square wave generation at P1.5 (speaker)
; 	Timer 1: Serial port baud rate 57600 generator
;  	Timer 2: 1ms interrupt for BCD counter increment/decrement
;----------------------------------------------------------------------------------------------;
; Routine to initialize the ISR for Timer 0 ;
Timer0_Init:
	mov a, TMOD
	anl a, #0xf0 ; Clear the bits for timer 0
	orl a, #0x01 ; Configure timer 0 as 16-timer
	mov TMOD, a
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
    setb ET0  ; Enable timer 0 interrupt
    setb TR0  ; Start timer 0
	ret

; ISR for timer 0.  Set to execute every 1/4096Hz 
; to generate a 2048 Hz square wave at pin P1.5 
Timer0_ISR:
	;clr TF0  ; According to the data sheet this is done for us already.
	mov TH0, #high(TIMER0_RELOAD) ; Timer 0 doesn't have autoreload in the CV-8052
	mov TL0, #low(TIMER0_RELOAD)
	cpl SOUND_OUT ; Connect speaker to P1.5
	reti
;-----------------------------------------------------------------------------------------------;

; -----------------------------------------------------------------------------------------------;
; Routine to initialize the serial port at 57600 baud (Timer 1 in mode 2)
Initialize_Serial_Port:
	; Configure serial port and baud rate
	clr TR1 ; Disable timer 1
	anl TMOD, #0x0f ; Mask the bits for timer 1
	orl TMOD, #0x20 ; Set timer 1 in 8-bit auto reload mode
    orl PCON, #80H ; Set SMOD to 1
	mov TH1, #low(TIMER_1_RELOAD)
	mov TL1, #low(TIMER_1_RELOAD) 
	setb TR1 ; Enable timer 1
	mov SCON, #52H
	ret

; uart sending functions
putchar:
	jbc	TI, putchar_L1
	sjmp putchar
putchar_L1:
	mov	SBUF,a
	ret

SendString:
    clr a
    movc a, @a+dptr
    jz SendString_L1
    lcall putchar
    inc dptr
    sjmp SendString  
SendString_L1:
	ret
; -----------------------------------------------------------------------------------------------;

;------------------------------------------------------------------------------------------------;
; Routine to initialize the ISR for timer 2 
Timer2_Init:
	mov T2CON, #0 ; Stop timer/counter.  Autoreload mode.
	mov TH2, #high(TIMER2_RELOAD)
	mov TL2, #low(TIMER2_RELOAD)
	; Set the reload value
	mov RCAP2H, #high(TIMER2_RELOAD)
	mov RCAP2L, #low(TIMER2_RELOAD)
	; Enable the timer and interrupts
    setb ET2  ; Enable timer 2 interrupt
    setb TR2  ; Enable timer 2
	ret

; ISR for timer 2.  Runs every 1 ms ;
Timer2_ISR:
	push acc
	push psw
	clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in ISR
	; cpl P1.1 ; Optional debug pin toggle for scope (ensure it's not used elsewhere)

; FSM states timers
	inc KEY1_DEB_timer
	inc SEC_FSM_timer
	
Timer2_ISR_done:
	pop psw
	pop acc
	reti
;-----------------------------------------------------------------------------------------------;

;-----------------------------------------------;
; Display Function fo 7-segment displays		;
;-----------------------------------------------;

; Look-up table for the 7-seg displays. (Segments are turn on with zero) 
T_7seg:
    DB 0xC0, 0xF9, 0xA4, 0xB0, 0x99        ; 0 TO 4
    DB 0x92, 0x82, 0xF8, 0x80, 0x90        ; 4 TO 9
    DB 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E  ; A to F

; Displays a BCD number pased in R0 in HEX5-HEX0
Display_BCD_7_Seg_HEX10:
	mov dptr, #T_7seg

	mov a, R0
	swap a
	anl a, #0FH
	movc a, @a+dptr
	mov HEX1, a
	
	mov a, R0
	anl a, #0FH
	movc a, @a+dptr
	mov HEX0, a
	
	ret

Display_BCD_7_Seg_HEX32:
	mov dptr, #T_7seg

	mov a, R0
	swap a
	anl a, #0FH
	movc a, @a+dptr
	mov HEX3, a
	
	mov a, R0
	anl a, #0FH
	movc a, @a+dptr
	mov HEX2, a
	
	ret

Display_BCD_7_Seg_HEX54:
	mov dptr, #T_7seg

	mov a, R0
	swap a
	anl a, #0FH
	movc a, @a+dptr
	mov HEX5, a
	
	mov a, R0
	anl a, #0FH
	movc a, @a+dptr
	mov HEX4, a
	
	ret

; The 8-bit hex number passed in the accumulator is converted to
; BCD and stored in [R1, R0]
Hex_to_bcd_8bit:
	mov b, #100
	div ab
	mov R1, a   ; After dividing, a has the 100s
	mov a, b    ; Remainder is in register b
	mov b, #10
	div ab ; The tens are stored in a, the units are stored in b 
	swap a
	anl a, #0xf0
	orl a, b
	mov R0, a
	ret

;-----------------------------------------------;
; Display Function fo LCD 						;
;-----------------------------------------------;




;---------------------------- -----;
;         Main program.           ;
;---------------------------------;
main:
	; -------------------------------------------------------------------;
	; Initialization
    mov SP, #0x7F
    lcall Timer0_Init
    lcall Timer2_Init
	lcall ELCD_4BIT
	lcall Initialize_Serial_Port

	; We use the pins of P0 to control the LCD.  Configure as outputs.
    mov P0MOD, #01111111b ; P0.0 to P0.6 are outputs.  ('1' makes the pin output)
    ; We use pins P1.5 and P1.1 as outputs also.  Configure accordingly.
    mov P1MOD, #00100010b ; P1.5 and P1.1 are outputs
    mov P2MOD, #0xff
    mov P3MOD, #0xff

    ; Turn off all the LEDs
    mov LEDRA, #0 ; LEDRA is bit addressable
    mov LEDRB, #0 ; LEDRB is NOT bit addresable

	; Enable Global interrupts
    setb EA  

	; FSM initial states
	mov KEY1_DEB_state, #0
	mov SEC_FSM_state, #0

	; FSM timers initialization
	mov KEY1_DEB_timer, #0
	mov SEC_FSM_timer, #0

	; Display initial message on LCD
	Set_Cursor(1, 1)
    Send_Constant_String(#Initial_Message)

	; Initialize counter to zero
    
	; -------------------------------------------------------------------;

loop:
;-------------------------------------------------------------------------------
; non-blocking state machine for KEY1 debounce
	mov a, KEY1_DEB_state
KEY1_DEB_state0:
	cjne a, #0, KEY1_DEB_state1
	jb KEY.1, KEY1_DEB_done
	mov KEY1_DEB_timer, #0
	inc KEY1_DEB_state
	sjmp KEY1_DEB_done
KEY1_DEB_state1:
	cjne a, #1, KEY1_DEB_state2
	; this is the debounce state
	mov a, KEY1_DEB_timer
	cjne a, #50, KEY1_DEB_done ; 50 ms passed?
	inc KEY1_DEB_state
	sjmp KEY1_DEB_done	
KEY1_DEB_state2:
	cjne a, #2, KEY1_DEB_state3
	jb KEY.1, KEY1_DEB_state2b
	inc KEY1_DEB_state
	sjmp KEY1_DEB_done	
KEY1_DEB_state2b:
	mov KEY1_DEB_state, #0
	sjmp KEY1_DEB_done
KEY1_DEB_state3:
	cjne a, #3, KEY1_DEB_done
	jnb KEY.1, KEY1_DEB_done
	setb Key1_flag ; Suscesfully detected a valid KEY1 press/release
	mov KEY1_DEB_state, #0	
KEY1_DEB_done:
;-------------------------------------------------------------------------------

;-------------------------------------------------------------------------------
; non-blocking FSM for the one second counter
	mov a, SEC_FSM_state
SEC_FSM_state0:
	cjne a, #0, SEC_FSM_state1
	mov a, SEC_FSM_timer
	cjne a, #250, SEC_FSM_done ; 250 ms passed?
	mov SEC_FSM_timer, #0
	inc SEC_FSM_state
	sjmp SEC_FSM_done
SEC_FSM_state1:	
	cjne a, #1, SEC_FSM_state2
	setb LEDRA.1
	mov a, SEC_FSM_timer
	cjne a, #250, SEC_FSM_done ; 250 ms passed?
	mov SEC_FSM_timer, #0
	inc SEC_FSM_state
	sjmp SEC_FSM_done
SEC_FSM_state2:	
	cjne a, #2, SEC_FSM_state3
	setb LEDRA.2
	mov a, SEC_FSM_timer
	cjne a, #250, SEC_FSM_done ; 250 ms passed?
	mov SEC_FSM_timer, #0
	inc SEC_FSM_state
	sjmp SEC_FSM_done
SEC_FSM_state3:	
	cjne a, #3, SEC_FSM_done
	setb LEDRA.3
	mov a, SEC_FSM_timer
	cjne a, #250, SEC_FSM_done ; 250 ms passed?
	mov SEC_FSM_timer, #0
	mov SEC_FSM_state, #0
	mov a, current_time_sec
	cjne a, #59, IncCurrentTimeSec ; Don't let the seconds counter pass 59
	mov current_time_sec, #0
	sjmp SEC_FSM_done
IncCurrentTimeSec:
	inc current_time_sec
	cpl LEDRA.0 ; 1 Hz heartbeat LED
SEC_FSM_done:
;-------------------------------------------------------------------------------

	ljmp loop



END
