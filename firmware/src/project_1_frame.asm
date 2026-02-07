; Project 1
; CV-8052 microcontroller in DE10-Lite board

; ----------------------------------------------------------------------------------------------;
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
; includes
$NOLIST
$include(..\inc\MODMAX10)
$include(..\inc\math32.asm) ; 
$LIST
; ----------------------------------------------------------------------------------------------;

; ----------------------------------------------------------------------------------------------;
; Data Segment 0x30 -- 0x7F  (overall 79d bytes available)
dseg at 0x30
current_time_sec:     ds 1
current_time_minute:  ds 1

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

power_output:  ds 4 ;
pwm_counter: ds 4 ; counter for pwm (0-1500)

KEY1_DEB_timer: ds 1
SEC_FSM_timer:  ds 1
KEY1_DEB_state:    ds 1
SEC_FSM_state: 	   ds 1
Control_FSM_state: ds 1 
; 46d bytes used
; ---------------------------------------------------------------------------------------------;

; ---------------------------------------------------------------------------------------------;
; bit operation setb, clr, jb, and jnb
bseg
mf:		dbit 1 ; math32 sign
one_second_flag: dbit 1
one_ms_pwm_flag: dbit 1 ; one_millisecond_flag for pwm signal

soak_temp_reached: dbit 1
reflow_temp_reached: dbit 1
cooling_temp_reached: dbit 1

soak_time_reached: dbit 1
reflow_time_reached: dbit 1

reset_signal: dbit 1
stop_signal: dbit 1
start_signal: dbit 1
config_finish_signal: dbit 1

state_change_signal: dbit 1

Key1_flag: dbit 1
PB0_flag: dbit 1 ; start entire program
PB1_flag: dbit 1 ; start soak
PB2_flag: dbit 1 ; pause process
; 11 bits used
; ---------------------------------------------------------------------------------------------;

; ---------------------------------------------------------------------------------------------;
cseg
CLK            EQU 33333333 ; Microcontroller system crystal frequency in Hz
BAUD 		   EQU 57600

TIMER0_RATE    EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD  EQU ((65536-(CLK/(12*TIMER0_RATE)))) ; The prescaler in the CV-8052 is always 12 unlike the N76E003 where is selectable.

TIMER_1_RELOAD EQU (256-((2*CLK)/(12*32*BAUD)))

TIMER2_RATE    EQU 1000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD  EQU ((65536-(CLK/(12*TIMER2_RATE))))

PWM_PERIOD     EQU 1499 ; 1.5s period

SOUND_OUT      EQU P1.5 ; Pin connected to the speaker

PWM_OUT		   EQU P1.3 ; Pin connected to the ssr for outputing pwm signal

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

;                     1234567890123456 <-- 16 characters per line LCD
Initial_Message:  db 'initial message', 0
String_state0_1:  db 'Welcome        ', 0
String_state0_2:  db 'Press PB0      ', 0

;                       1234567890123456
String_state1:      db 'Set Parameters ', 0
String_soak_temp:   db 'Soak Temp:', 0
String_reflow_temp: db 'Reflow Temp:', 0
String_soak_time:   db 'Soak Time:', 0
String_reflow_time: db 'Reflow Time:', 0

;                     1234567890123456
String_state2:    db 'Ramp to Soak   ', 0
String_state3:    db 'Soak Phase     ', 0
String_state4:    db 'Ramp to Reflow ', 0
String_state5:    db 'Reflow Phase   ', 0
String_state6:    db 'Cooling        ', 0
String_state7:    db 'Process Done   ', 0

String_Blank:    db '                ', 0


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

; serial debugging
; send a four byte number via serial to laptop
; need to be used with python script
; content needed to be sent should be stored in the varaible x
Send32:
    ; data format: 0xAA, 0x55, x+3, x+2, x+1, x+0, 0xAH (big endian)
    mov A, #0AAH
    lcall putchar
    mov A, #055H
    lcall putchar

    mov A, x+3
    lcall putchar
    mov A, x+2
    lcall putchar
    mov A, x+1
    lcall putchar
    mov A, x+0
    lcall putchar

    mov A, #0AH
    lcall putchar
    ret
; -----------------------------------------------------------------------------------------------;

;-----------------------------------------------------------------------------------------------;
$include(..\inc\Timer2_ISR.inc) ; Timer 2 ISR for 1ms tick and pwm signal generation
$include(..\inc\LCD_4bit_DE10Lite_no_RW.inc) ; LCD related functions and utility macros
;-----------------------------------------------------------------------------------------------;

;-----------------------------------------------------------------------------------------------;
; Display Function for 7-segment displays		

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

;------------------------------------------------------------------;
; Display Function for LCD 						
LCD_Display_Update_func:
	push acc
	jnb state_change_signal, LCD_Display_Update_Done
	clr state_change_signal
	mov a, Control_FSM_state

LCD_Display_Update_0:
	cjne a, #0, LCD_Display_Update_1
	Set_Cursor(1,1)
	Send_Constant_String(#String_state0_1)
	Set_Cursor(2,1)
	Send_Constant_String(#String_state0_2)
	ljmp LCD_Display_Update_done

LCD_Display_Update_1:
	cjne a, #1, LCD_Display_Update_2
	Set_Cursor(1,1)
	Send_Constant_String(#String_state1)
	ljmp LCD_Display_Update_done

LCD_Display_Update_2:
	cjne a, #2, LCD_Display_Update_3
	Set_Cursor(1,1)
	Send_Constant_String(#String_state2)
	ljmp LCD_Display_Update_done

LCD_Display_Update_3:
	cjne a, #3, LCD_Display_Update_4
	Set_Cursor(1,1)
	Send_Constant_String(#String_state3)
	ljmp LCD_Display_Update_done

LCD_Display_Update_4:
	cjne a, #4, LCD_Display_Update_5
	Set_Cursor(1,1)
	Send_Constant_String(#String_state4)
	ljmp LCD_Display_Update_done

LCD_Display_Update_5:
	cjne a, #5, LCD_Display_Update_6
	Set_Cursor(1,1)
	Send_Constant_String(#String_state5)
	ljmp LCD_Display_Update_done

LCD_Display_Update_6:
	cjne a, #6, LCD_Display_Update_7
	Set_Cursor(1,1)
	Send_Constant_String(#String_state6)
	ljmp LCD_Display_Update_done

LCD_Display_Update_7:
	cjne a, #7, LCD_Display_Update_done
	Set_Cursor(1,1)
	Send_Constant_String(#String_state7)
	ljmp LCD_Display_Update_done

LCD_Display_Update_done:
	pop acc
	ret

;-------------------------------------------------------------------------------
; non-blocking state machine for KEY1 debounce
KEY1_DEB:
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
	ret
; ------------------------------------------------------------------------------

;-------------------------------------------------------------------------------
; non-blocking FSM for the one second counter
SEC_FSM:
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
	ret

;-------------------------------------------------------------------------------
; PWM
; generate pwm signal for the ssr ; 1.5s period for the pwm signal; with 1 watt 
; clarity for the pwm signal; input parameter: power_output; used buffers: x, y
PWM_Wave: ; call pwm generator when 1 ms flag is triggered
	jbc one_ms_pwm_flag, pwm_wave_generator
	sjmp end_pwm_generator

pwm_wave_generator:
	clr mf
	; move pwm counter value into x for comparison purpose
	mov x, pwm_counter
	mov x+1, pwm_counter+1
	mov x+2, pwm_counter+2
	mov x+3, pwm_counter+3

	Load_Y(PWM_PERIOD)

	; compare x(pwm_counter) and y(1499) if x=y, wrap x back to 0; else increase x by 1
	lcall x_eq_y 
	jb mf, wrap_pwm_counter
	; x not equal 1499, increment by 1
	Load_Y(1)
	lcall add32
	; update pwm_counter
	mov pwm_counter, x
	mov pwm_counter+1, x+1
	mov pwm_counter+2, x+2
	mov pwm_counter+3, x+3
	sjmp set_pwm

wrap_pwm_counter:
	; x equal 1499, wrap to 0
	Load_X(0)
	mov pwm_counter, x
	mov pwm_counter+1, x+1
	mov pwm_counter+2, x+2
	mov pwm_counter+3, x+3

set_pwm:
	; compare with power_output, if pwm counter smaller than power_output, set pwm pin high; else set pwm pin low
	; load y with power output value
	mov y, power_output
	mov y+1, power_output+1
	mov y+2, power_output+2
	mov y+3, power_output+3

	; compare x(pwm counter) with y(power output)
	lcall x_lt_y
	jb mf, set_pwm_high ; set pwm pin high if pwm counter smaller than power output
	; set pwm pin low if pwm counter greater than power output
	clr PWM_OUT
	clr LEDRA.4
	sjmp end_pwm_generator

set_pwm_high:
	setb PWM_OUT
	setb LEDRA.4

end_pwm_generator:
	ret
;-------------------------------------------------------------------------------;

;-------------------------------------------------------------------------------;
; main control fsm for the entire process
Control_FSM:
	mov a, Control_FSM_state
	sjmp Control_FSM_state0

Control_FSM_state0_a:
	mov Control_FSM_state, #0
	setb state_change_signal
Control_FSM_state0:
	cjne a, #0, Control_FSM_state1
	jbc PB0_flag, Control_FSM_state1_a
	sjmp Control_FSM_done

Control_FSM_state1_a:
	inc Control_FSM_state
	setb state_change_signal
Control_FSM_state1:
	cjne a, #1, Control_FSM_state2
	jbc PB1_flag, Control_FSM_state1_b
	sjmp Control_FSM_done
Control_FSM_state1_b:
	jbc config_finish_signal, Control_FSM_state2_a
	sjmp Control_FSM_done

Control_FSM_state2_a:
	inc Control_FSM_state
	setb state_change_signal
Control_FSM_state2:
	cjne a, #2, Control_FSM_state3
	jbc PB2_flag, Control_FSM_state6_a
	jbc soak_temp_reached, Control_FSM_state3_a
	sjmp Control_FSM_done

Control_FSM_state3_a:
	inc Control_FSM_state
	setb state_change_signal
Control_FSM_state3:
	cjne a, #3, Control_FSM_state4
	jbc PB2_flag, Control_FSM_state6_a
	jbc soak_time_reached, Control_FSM_state4_a
	sjmp Control_FSM_done

Control_FSM_state4_a:
	inc Control_FSM_state	
	setb state_change_signal
Control_FSM_state4:
	cjne a, #4, Control_FSM_state5
	jbc PB2_flag, Control_FSM_state6_a
	jbc reflow_temp_reached, Control_FSM_state5_a
	sjmp Control_FSM_done

Control_FSM_state5_a:
	inc Control_FSM_state
	setb state_change_signal
Control_FSM_state5:
	cjne a, #5, Control_FSM_state6
	jbc PB2_flag, Control_FSM_state6_a
	jbc reflow_time_reached, Control_FSM_state6_a
	sjmp Control_FSM_done

Control_FSM_state6_a:
	inc Control_FSM_state
	setb state_change_signal
Control_FSM_state6:
	cjne a, #6, Control_FSM_done
	jbc cooling_temp_reached, Control_FSM_state7_a
	sjmp Control_FSM_done

Control_FSM_state7_a:
	inc Control_FSM_state
	setb state_change_signal
Control_FSM_state7:
	cjne a, #7, Control_FSM_done
	jbc PB0_flag, Control_FSM_state0_a
	sjmp Control_FSM_done

Control_FSM_done:
	ret
;-------------------------------------------------------------------------------;


;---------------------------------;
;         Main program.           ;
;---------------------------------;
main:
	; -------------------------------------------------------------------;
	; Initialization
    mov SP, #0x7F

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
	mov Control_FSM_state, #0
	; FSM timers initialization
	mov KEY1_DEB_timer, #0
	mov SEC_FSM_timer, #0
	; time counters initialization
	mov current_time_sec, #0
	mov current_time_minute, #0
	; Initialize counter to zero
    mov pwm_counter, #0
	mov pwm_counter+1, #0
	mov pwm_counter+2, #0
	mov pwm_counter+3, #0
	; Initialize power output
	mov power_output+3, #0
	mov power_output+2, #0
	mov power_output+1, #02H
	mov power_output, #0EEH ; (initilize to 750 for testing)

	; Clear all the flags
	clr PB0_flag
	clr PB1_flag
	clr PB2_flag
	clr one_second_flag
	clr config_finish_signal
	clr soak_temp_reached
	clr soak_time_reached
	clr reflow_temp_reached
	clr reflow_time_reached
	clr cooling_temp_reached
	clr state_change_signal

	lcall Timer0_Init
    lcall Timer2_Init
	lcall ELCD_4BIT
	lcall Initialize_Serial_Port

loop:
	; Check the FSM for KEY1 debounce
	lcall KEY1_DEB

	; Check the FSM for one second counter
	lcall SEC_FSM

	; Check the FSM for the overall control flow of the reflow process
	lcall Control_FSM

	; Update the LCD display based on the current state
	lcall LCD_Display_Update_func

	; Update the pwm output for the ssr
	lcall PWM_Wave 

	; After initialization the program stays in this 'forever' loop
	ljmp loop

END
