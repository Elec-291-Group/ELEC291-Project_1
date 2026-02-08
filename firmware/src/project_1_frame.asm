; Project 1
; CV-8052 microcontroller in DE10-Lite board
;-------------------------------------------------------------------------------
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
;-------------------------------------------------------------------------------
; includes
$NOLIST
$include(..\inc\MODMAX10)
$include(..\inc\LCD_4bit_DE10Lite_no_RW.inc) ; LCD related functions and utility macros
$include(..\inc\math32.asm) ; 
$LIST
; ----------------------------------------------------------------------------------------------;
; Data Segment 0x30 -- 0x7F  (overall 79d bytes available)
dseg at 0x30
current_time_sec:     ds 1
current_time_minute:  ds 1
soak_time_sec:        ds 1
soak_time_minute:     ds 1
reflow_time_sec:      ds 1
reflow_time_minute:   ds 1

; math32 buffer variables
x:		ds	4
y:		ds	4
bcd:	ds	5

current_temp: ds 4 ;
soak_temp:    ds 4 ;
reflow_temp:  ds 4 ;

power_output:  ds 4 ;
pwm_counter: ds 4 ; counter for pwm (0-1500)

KEY1_DEB_timer: ds 1
SEC_FSM_timer:  ds 1
KEY1_DEB_state:    ds 1
SEC_FSM_state: 	   ds 1
Control_FSM_state: ds 1 

;------------------------------
; Buzzer module variables
;------------------------------
buzz_state:      ds 1   ; 0=IDLE, 1=ON, 2=OFF
buzz_timer:      ds 1   ; counts ms within ON/OFF window
buzz_beeps_left: ds 1   ; how many beeps remaining
buzz_priority:   ds 1   ; 0 none, 1=state, 2=done, 3=error

; 46d bytes used

;-------------------------------------------------------------------------------
; bit operation setb, clr, jb, and jnb
bseg
mf:		dbit 1 ; math32 sign
one_second_flag: dbit 1
one_millisecond_flag: dbit 1 ; one_millisecond_flag for pwm signal
one_millisecond_flag_buzz: dbit 1 ; one_millisecond_flag for buzz

soak_temp_reached: dbit 1
reflow_temp_reached: dbit 1
cooling_temp_reached: dbit 1

soak_time_reached: dbit 1
reflow_time_reached: dbit 1

reset_signal: dbit 1
stop_signal: dbit 1
start_signal: dbit 1
config_finish_signal: dbit 1

Key1_flag: dbit 1

tc_missing_abort: dbit 1   ; 1 = abort because temp < 50C after 60s
tc_startup_window: dbit 1   ; 1 = still within first 60 seconds of the run
PB0_flag: dbit 1 ; start entire program
PB1_flag: dbit 1 ; start soak
PB2_flag: dbit 1 ; pause process
; 11 bits used

;-------------------------------------------------------------------------------
cseg
CLK            EQU 33333333 ; Microcontroller system crystal frequency in Hz
BAUD 		   EQU 57600

TIMER0_RATE    EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD  EQU ((65536-(CLK/(12*TIMER0_RATE)))) ; The prescaler in the CV-8052 
; is always 12 unlike the N76E003 where is selectable.

TIMER_1_RELOAD EQU (256-((2*CLK)/(12*32*BAUD)))

TIMER2_RATE    EQU 1000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD  EQU ((65536-(CLK/(12*TIMER2_RATE))))

PWM_PERIOD     EQU 1499 ; 1.5s period
BEEP_ON_MS	   EQU 100  ; 100ms
BEEP_OFF_MS    EQU 100  ; 100ms


SOUND_OUT      EQU P1.5 ; Pin connected to the speaker

PWM_OUT		   EQU P1.3 ; Pin connected to the ssr for outputing pwm signal

; These 'equ' must match the wiring between the DE10Lite board and the LCD!
; P0 is in connector JPIO.
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

;-------------------------------------------------------------------------------
; Timers Setting:
;   Timer 0: 2kHz square wave generation at P1.5 (speaker)
; 	Timer 1: Serial port baud rate 57600 generator
;  	Timer 2: 1ms interrupt for BCD counter increment/decrement
;-------------------------------------------------------------------------------
; Routine to initialize the ISR for Timer 0 ;
Timer0_Init:
	mov a, TMOD
	anl a, #0xf0 ; Clear the bits for timer 0
	orl a, #0x01 ; Configure timer 0 as 16-timer
	mov TMOD, a
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
    setb ET0        ; Enable timer 0 interrupt
    clr  TR0        ; DO NOT start tone by default (buzzer task controls this)
    clr  SOUND_OUT  ; keep buzzer pin low when off
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

;-------------------------------------------------------------------------------
; serial debugging
; send a four byte number via serial to laptop
; need to be used with python script
; content needed to be sent should be stored in the varaible x
;-------------------------------------------------------------------------------
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

;-------------------------------------------------------------------------------
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

	setb one_millisecond_flag ; set the one millisecond flag
	setb one_millisecond_flag_buzz ; set the one millisecond flag for buzz

Timer2_ISR_done:
	pop psw
	pop acc
	reti

;-------------------------------------------------------------------------------
; Display Function for 7-segment displays		
;-------------------------------------------------------------------------------
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

;-------------------------------------------------------------------------------
; Display Function for LCD 						
;-------------------------------------------------------------------------------
LCD_Display_Update_func:
	push acc
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
;---------------------------------------------------------

KEY1_DEB:
;non-blocking state machine for KEY1 debounce
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
; Non-blocking FSM for the one second counter
;-------------------------------------------------------------------------------
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
    cjne a, #59, IncCurrentTimeSec

    ; seconds reached 59 -> wrap to 0 and increment minutes
    mov current_time_sec, #0
    inc current_time_minute
    cpl LEDRA.0
    sjmp SEC_FSM_done

IncCurrentTimeSec:
    inc current_time_sec
    cpl LEDRA.0

SEC_FSM_done:
	ret

;-------------------------------------------------------------------------------
; PWM
; generate pwm signal for the ssr ; 1.5s period for the pwm signal; with 1 watt 
; clarity for the pwm signal; input parameter: power_output; used buffers: x, y
; ------------------------------------------------------------------------------
PWM_Wave: ; call pwm generator when 1 ms flag is triggered
	jbc one_millisecond_flag, pwm_wave_generator
	sjmp end_pwm_generator

pwm_wave_generator:
	clr one_millisecond_flag
	clr mf
	; move pwm counter value into x for comparison purpose
	mov x, pwm_counter
	mov x+1, pwm_counter+1
	mov x+2, pwm_counter+2
	mov x+3, pwm_counter+3

	Load_Y(PWM_PERIOD)

	; compare x(pwm_counter) and y(1499) if x=y, wrap x back to 0; else 
	; increase x by 1
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
	; compare with power_output, if pwm counter smaller than power_output, 
	; set pwm pin high; else set pwm pin low load y with power output value
	mov y, power_output
	mov y+1, power_output+1
	mov y+2, power_output+2
	mov y+3, power_output+3

	; compare x(pwm counter) with y(power output)
	lcall x_lt_y
	jb mf, set_pwm_high ; set pwm pin high if pwm counter smaller than power 
	;output set pwm pin low if pwm counter greater than power output
	clr PWM_OUT
	clr LEDRA.4
	sjmp end_pwm_generator

set_pwm_high:
	setb PWM_OUT
	setb LEDRA.4

end_pwm_generator:
	ret

;-------------------------------------------------------------------------------
; BUZZER MODULE (non-blocking)
; Public API:
;   lcall Beep_StateChange   ; 1 beep  (priority 1)
;   lcall Beep_Done          ; 5 beeps (priority 2)
;   lcall Beep_Error         ; 10 beeps(priority 3)
; Background task:
;   lcall Buzzer_Task        ; call every loop
;-------------------------------------------------------------------------------

Beep_StateChange:
    mov  B,  #1
    mov  R7, #1
    lcall Beep_Request
    ret

Beep_Done:
    mov  B,  #5
    mov  R7, #2
    lcall Beep_Request
    ret

Beep_Error:
    mov  B,  #10
    mov  R7, #3
    lcall Beep_Request
    ret

;---------------------------------------------------------
; Beep_Request
; Inputs:
;   B  = number of beeps
;   R7 = priority (1/2/3)
; Behavior:
;   If new priority >= current, override current pattern.
;---------------------------------------------------------
Beep_Request:
    push acc
    push psw

    mov  a, buzz_priority
    clr  c
    subb a, R7
    jc   Beep_Accept
    jz   Beep_Accept
    sjmp Beep_Req_Done

Beep_Accept:
    mov  buzz_priority, R7
    mov  buzz_beeps_left, B
    mov  buzz_timer, #0
    mov  buzz_state, #1      ; start ON now
    setb TR0                 ; tone ON

Beep_Req_Done:
    pop  psw
    pop  acc
    ret

;---------------------------------------------------------
; Buzzer_Task
; - Call every loop
; - Advances only when one_millisecond_flag is set
;---------------------------------------------------------
Buzzer_Task:
    jbc  one_millisecond_flag_buzz, Buzzer_Step
    ret

Buzzer_Step:
    mov  a, buzz_state

; IDLE
Buzzer_State0:
    cjne a, #0, Buzzer_State1
    clr  TR0
    clr  SOUND_OUT
    mov  buzz_priority, #0
    ret

; ON phase
Buzzer_State1:
    cjne a, #1, Buzzer_State2
    inc  buzz_timer
    mov  a, buzz_timer
    cjne a, #BEEP_ON_MS, Buzzer_Done

    ; ON time elapsed -> go OFF
    mov  buzz_timer, #0
    mov  buzz_state, #2
    clr  TR0
    clr  SOUND_OUT
    ret

; OFF phase
Buzzer_State2:
    inc  buzz_timer
    mov  a, buzz_timer
    cjne a, #BEEP_OFF_MS, Buzzer_Done

    ; OFF time elapsed -> one beep finished
    mov  buzz_timer, #0

    mov  a, buzz_beeps_left
    jz   Buzzer_GoIdle
    dec  buzz_beeps_left
    mov  a, buzz_beeps_left
    jz   Buzzer_GoIdle

    ; start next beep
    mov  buzz_state, #1
    setb TR0
    ret

Buzzer_GoIdle:
    mov  buzz_state, #0
    clr  TR0
    clr  SOUND_OUT
    mov  buzz_priority, #0
    ret

Buzzer_Done:
    ret
;---------------------------------------------------------

;-------------------------------------------------------------------------------
; Time_Compare_MMSS
;
; PURPOSE:
;   Compare elapsed time (current_time_minute:current_time_sec)
;   against soak and reflow setpoints (soak_time_*, reflow_time_*).
;
; BEHAVIOR:
;   If current >= soak   -> set soak_time_reached
;   If current >= reflow -> set reflow_time_reached
;
; NOTES:
;   Compare minutes first, then seconds.
;-------------------------------------------------------------------------------
Time_Compare_MMSS:
    push acc
    push psw

;-----------------------------
; current >= soak ?
;-----------------------------
    mov  a, current_time_minute
    clr  c
    subb a, soak_time_minute
    jc   TC_Soak_NotReached      ; current_min < soak_min
    jnz  TC_Soak_Reached         ; current_min > soak_min

    ; minutes equal -> compare seconds
    mov  a, current_time_sec
    clr  c
    subb a, soak_time_sec
    jc   TC_Soak_NotReached      ; current_sec < soak_sec

TC_Soak_Reached:
    setb soak_time_reached

TC_Soak_NotReached:

;-----------------------------
; current >= reflow ?
;-----------------------------
    mov  a, current_time_minute
    clr  c
    subb a, reflow_time_minute
    jc   TC_Reflow_NotReached
    jnz  TC_Reflow_Reached

    mov  a, current_time_sec
    clr  c
    subb a, reflow_time_sec
    jc   TC_Reflow_NotReached

TC_Reflow_Reached:
    setb reflow_time_reached

TC_Reflow_NotReached:
    pop  psw
    pop  acc
    ret

;-------------------------------------------------------------------------------;
; Copy4_Bytes_R0_to_R1
;
; PURPOSE:
;   Utility routine to copy a 32-bit value (4 bytes)
;   from one memory location to another.
;
; INPUTS:
;   R0 st source address
;   R1 at destination address
;
; USES:
;   R2 as loop counter
;
; EXAMPLE:
;   mov R0, #current_temp
;   mov R1, #x
;   lcall Copy4_Bytes_R0_to_R1
;-------------------------------------------------------------------------------;
Copy4_Bytes_R0_to_R1:
    mov  R2, #4
Copy4_Loop:
    mov  a, @R0
    mov  @R1, a
    inc  R0
    inc  R1
    djnz R2, Copy4_Loop
    ret
;-------------------------------------------------------------------------------;
; Abort condition safety check Temperature time
;
; PURPOSE:
;   Automatic cycle termination on error:
;   Abort if oven fails to reach at least 50C in first 60s.
;
; TRIP CONDITION:
;   if (current_time >= 60s) AND (current_temp < 50C)
;       -> set tc_missing_abort
;       -> set stop_signal
;
; ASSUMPTIONS:
;   - current_time is in SECONDS (32-bit, little-endian)
;   - current_temp is in DEGREES C (integer, 32-bit, little-endian)
;
;   the Load_Y constants accordingly.
;-------------------------------------------------------------------------------;

Safety_Check_TC:
    push acc
    push psw
    push AR0
    push AR1
    push AR2

    ; If already aborted or startup window closed, do nothing
    jb   tc_missing_abort, Safety_TC_Done
    jnb  tc_startup_window, Safety_TC_Done

    ; Check: elapsed >= 60 seconds ?
    mov  a, current_time_minute
    jnz  Safety_TC_At60          ; if minute >= 1, definitely >=60s

    mov  a, current_time_sec
    clr  c
    subb a, #60
    jc   Safety_TC_Done          ; still < 60s

Safety_TC_At60:

    ; We reached 60s: close the startup window so it won't re-check later
    clr  tc_startup_window

    ; Now check: current_temp < 50 ?
    mov  R0, #current_temp
    mov  R1, #x
    lcall Copy4_Bytes_R0_to_R1

    Load_Y(50)
    lcall x_lt_y
    jnb  mf, Safety_TC_Done        ; temp >= 50 → pass

    ; FAIL: at 60s, still below 50C → abort
    setb tc_missing_abort
    setb stop_signal
    clr  PWM_OUT

Safety_TC_Done:
    pop  AR2
    pop  AR1
    pop  AR0
    pop  psw
    pop  acc
    ret
;-------------------------------------------------------------------------------;
; Main Control FSM for the entire process
;-------------------------------------------------------------------------------;
Control_FSM:
	mov a, Control_FSM_state
	sjmp Control_FSM_state0

Control_FSM_state0_a:
	mov Control_FSM_state, #0
Control_FSM_state0:
	cjne a, #0, Control_FSM_state1
	jbc PB0_flag, Control_FSM_state1_a
	sjmp Control_FSM_done

Control_FSM_state1_a:
	inc Control_FSM_state
Control_FSM_state1:
	cjne a, #1, Control_FSM_state2
	jbc PB1_flag, Control_FSM_state1_b
	sjmp Control_FSM_done
Control_FSM_state1_b:
	jbc config_finish_signal, Control_FSM_state2_a
	sjmp Control_FSM_done

Control_FSM_state2_a:
	inc Control_FSM_state
Control_FSM_state2:
	cjne a, #2, Control_FSM_state3
	jbc PB2_flag, Control_FSM_state6_a
	jbc soak_temp_reached, Control_FSM_state3_a
	sjmp Control_FSM_done

Control_FSM_state3_a:
	inc Control_FSM_state
Control_FSM_state3:
	cjne a, #3, Control_FSM_state4
	jbc PB2_flag, Control_FSM_state6_a
	jbc soak_time_reached, Control_FSM_state4_a
	sjmp Control_FSM_done

Control_FSM_state4_a:
	inc Control_FSM_state	
Control_FSM_state4:
	cjne a, #4, Control_FSM_state5
	jbc PB2_flag, Control_FSM_state6_a
	jbc reflow_temp_reached, Control_FSM_state5_a
	sjmp Control_FSM_done

Control_FSM_state5_a:
	inc Control_FSM_state
Control_FSM_state5:
	cjne a, #5, Control_FSM_state6
	jbc PB2_flag, Control_FSM_state6_a
	jbc reflow_time_reached, Control_FSM_state6_a
	sjmp Control_FSM_done

Control_FSM_state6_a:
	inc Control_FSM_state
Control_FSM_state6:
	cjne a, #6, Control_FSM_done
	jbc cooling_temp_reached, Control_FSM_state7_a
	sjmp Control_FSM_done

Control_FSM_state7_a:
	inc Control_FSM_state
Control_FSM_state7:
	cjne a, #7, Control_FSM_done
	jbc PB0_flag, Control_FSM_state0_a
	sjmp Control_FSM_done

Control_FSM_done:
	ret
;-------------------------------------------------------------------------------;
;         Main program.          
;-------------------------------------------------------------------------------;
main:
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
	mov soak_time_sec, #0
    mov soak_time_minute, #0
    mov reflow_time_sec, #0
    mov reflow_time_minute, #0

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
	; Initialize Buzzer
    mov buzz_state, #0
    mov buzz_timer, #0
    mov buzz_beeps_left, #0
    mov buzz_priority, #0

	; Clear all the flags
	clr  tc_missing_abort
	clr  stop_signal
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
    clr TR0
    clr SOUND_OUT

	; Set bit
	setb tc_startup_window

	lcall Timer0_Init
    lcall Timer2_Init
	lcall ELCD_4BIT
	lcall Initialize_Serial_Port
;-------------------------------------------------------------------------------;
; while(1) loop
;-------------------------------------------------------------------------------;
loop:
	; Check the FSM for KEY1 debounce
	lcall KEY1_DEB

	; Check the FSM for one second counter
	lcall SEC_FSM

	; Check the time compare
    lcall Time_Compare_MMSS
    lcall Safety_Check_TC

	; Check the temp compare
    lcall Temp_Compare

	; Check the FSM for the overall control flow of the reflow process
	lcall Control_FSM

	; Update the LCD display based on the current state
	;lcall LCD_Display_Update_func

	; Update the pwm output for the ssr
	lcall PWM_Wave 

    ; Run buzzer background task
    lcall Buzzer_Task

	; After initialization the program stays in this 'forever' loop
	ljmp loop
;-------------------------------------------------------------------------------;

END
