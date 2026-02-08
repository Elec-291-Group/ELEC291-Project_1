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
$include(..\inc\math32.asm) ; 
$LIST
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

Current_State:     ds 1


;-- UI buffers I added (ayaan)
Cursor_Idx: ds 1

; These hold the TEXT (ASCII) safely
; Digits Only + Null Terminator, got rid of C,:, and s 
Buf_Soak_Temp: ds 4   
Buf_Soak_Time: ds 5   
Buf_Refl_Temp: ds 4   
Buf_Refl_Time: ds 5

; 46d bytes used

;-------------------------------------------------------------------------------
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

SOUND_OUT      EQU P1.5 ; Pin connected to the speaker

PWM_OUT		   EQU P1.3 ; Pin connected to the ssr for outputing pwm signal

; These 'equ' must match the wiring between the DE10Lite board and the LCD!
; P0 is in connector JPIO.

;Added correct I/O definitions
;-- LCD Pins ---
ELCD_RS equ P1.7
ELCD_E  equ P1.1
ELCD_D4 equ P0.7
ELCD_D5 equ P0.5
ELCD_D6 equ P0.3
ELCD_D7 equ P0.1

; -- Buttons --
BTN_SOAK_TEMP equ P0.0
BTN_SOAK_TIME equ P0.2
BTN_REFL_TEMP equ P0.4
BTN_REFL_TIME equ P0.6

; --- KEYPAD ---
ROW1 equ P1.2
ROW2 equ P1.4
ROW3 equ P1.6
ROW4 equ P2.0
COL1 equ P2.2
COL2 equ P2.4
COL3 equ P2.6
COL4 equ P3.0

;                     1234567890123456 <-- 16 characters per line LCD
Initial_Message:  db 'initial message', 0
String_state0_1:  db 'Welcome        ', 0
String_state0_2:  db 'Press PB0      ', 0

; --- UI STRINGS (REQUIRED FOR KEYPAD LOGIC), <- I can fix if duplicates
Txt_Home:     db 'Select Mode:    ', 0
Txt_SoakT:    db 'Set Soak Temp   ', 0
Txt_SoakTime: db 'Set Soak Time   ', 0
Txt_ReflT:    db 'Set Reflow Temp ', 0
Txt_ReflTime: db 'Set Reflow Time ', 0

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
; -----------------------------------------------------------------------------------------------;

;-----------------------------------------------------------------------------------------------;
$include(..\inc\Timer2_ISR.inc) ; Timer 2 ISR for 1ms tick and pwm signal generation
$include(..\inc\LCD_4bit_DE10Lite_no_RW.inc) ; LCD related functions and utility macros
;-----------------------------------------------------------------------------------------------;

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
	
    ; --- FIX: JUMP DISTANCE ERROR ---
    ; "LCD_Display_Update_Done" is too far away for a JNB instruction.
    ; We use a local label right here to exit quickly.
	jnb state_change_signal, LCD_Local_Exit
    ; --------------------------------

	clr state_change_signal
	mov a, Control_FSM_state

	; --- IMPORTANT ADD ----
    ; If we are in State 1 (Setup), DO NOT RUN THIS, let the keypad logic handle the screen
    cjne a, #1, LCD_Display_Update_0
    pop acc
    ret 

; --- NEW LOCAL EXIT LABEL ---
LCD_Local_Exit:
    pop acc
    ret
; ----------------------------

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

LCD_Display_Update_Temp:
	
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
; ------------------------------------------------------------------------------
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

;-------------------------------------------------------------------------------;
; Temp_Compare
;
; PURPOSE:
;   Compare the current measured temperature against
;   the soak and reflow temperature setpoints.
;
; BEHAVIOR:
;   - If current_temp >= soak_temp   if soak_temp_reached   = 1
;   - If current_temp >= reflow_temp if reflow_temp_reached = 1
;
; NOTES:
;   - Uses 32-bit UNSIGNED comparison from math32.asm
;   - Comparison is done by:
;       x < y ?   (mf = 1)  if NOT reached
;       x >= y ?  (mf = 0)  if reached
;   - This routine ONLY SETS flags.
;     Clearing flags must be handled by the FSM.
;
; EXPECTED VARIABLES (DSEG / BSEG):
;   current_temp[4], soak_temp[4], reflow_temp[4]
;   x[4], y[4]
;   mf (math32 compare flag)
;   soak_temp_reached, reflow_temp_reached
;-------------------------------------------------------------------------------;
Temp_Compare:
    push acc
    push psw
    push AR0
    push AR1
    push AR2
    
; Check: current_temp >= soak_temp ?
    ; Copy current_temp of x (math32 operand A)
    mov  R0, #current_temp
    mov  R1, #x
    lcall Copy4_Bytes_R0_to_R1

    ; Copy soak_temp of y (math32 operand B)
    mov  R0, #soak_temp
    mov  R1, #y
    lcall Copy4_Bytes_R0_to_R1

    ; Perform x < y comparison
    ; mf = 1 if current_temp < soak_temp  (NOT reached)
    ; mf = 0 if current_temp >= soak_temp (REACHED)
    lcall x_lt_y
    jb   mf, Temp_Soak_NotReached
    setb soak_temp_reached

; Check: current_temp >= reflow_temp ?
Temp_Soak_NotReached:
    ; Copy current_temp of x
    mov  R0, #current_temp
    mov  R1, #x
    lcall Copy4_Bytes_R0_to_R1

    ; Copy reflow_temp of y
    mov  R0, #reflow_temp
    mov  R1, #y
    lcall Copy4_Bytes_R0_to_R1

    ; Compare x < y again
    lcall x_lt_y
    jb   mf, Temp_Reflow_NotReached
    setb reflow_temp_reached

Temp_Reflow_NotReached:
    pop  AR2
    pop  AR1
    pop  AR0
    pop  psw
    pop  acc
    ret
;-------------------------------------------------------------------------------;
; Time_Compare
;
; PURPOSE:
;   Compare the elapsed time against soak and reflow
;   time limits.
;
; BEHAVIOR:
;   - If current_time >= soak_time   if soak_time_reached   = 1
;   - If current_time >= reflow_time if reflow_time_reached = 1
;
; NOTES:
;   - Time values are treated as 32-bit UNSIGNED numbers
;     (e.g., milliseconds or seconds).
;   - Uses the SAME compare logic as Temp_Compare.
;   - This routine ONLY SETS flags.
;
; EXPECTED VARIABLES:
;   current_time[4], soak_time[4], reflow_time[4]
;   x[4], y[4]
;   mf, soak_time_reached, reflow_time_reached
;-------------------------------------------------------------------------------;
Time_Compare:
    push acc
    push psw
    push AR0
    push AR1
    push AR2

; Check: current_time >= soak_time ?
    ; Copy current_time of x
    mov  R0, #current_time
    mov  R1, #x
    lcall Copy4_Bytes_R0_to_R1

    ; Copy soak_time of y
    mov  R0, #soak_time
    mov  R1, #y
    lcall Copy4_Bytes_R0_to_R1

    ; Compare elapsed time vs soak time
    lcall x_lt_y
    jb   mf, Time_Soak_NotReached
    setb soak_time_reached

; Check: current_time >= reflow_time ?
Time_Soak_NotReached:
    ; Copy current_time of x
    mov  R0, #current_time
    mov  R1, #x
    lcall Copy4_Bytes_R0_to_R1

    ; Copy reflow_time of y
    mov  R0, #reflow_time
    mov  R1, #y
    lcall Copy4_Bytes_R0_to_R1

    ; Compare elapsed time vs reflow time
    lcall x_lt_y
    jb   mf, Time_Reflow_NotReached
    setb reflow_time_reached

Time_Reflow_NotReached:
    pop  AR2
    pop  AR1
    pop  AR0
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

    ; Check: current_time >= 60 ?
    mov  R0, #current_time
    mov  R1, #x
    lcall Copy4_Bytes_R0_to_R1

    Load_Y(60)
    lcall x_lt_y
    jb   mf, Safety_TC_Done        ; still < 60s → keep waiting

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
	setb state_change_signal
Control_FSM_state0:
	cjne a, #0, Control_FSM_state1
	jb P1.0, Control_FSM_done
	lcall Wait_For_P1_0_Release
	sjmp Control_FSM_state1_a

Control_FSM_state1_a:
	inc Control_FSM_state
	setb state_change_signal
Control_FSM_state1:
	cjne a, #1, Control_FSM_state2
	; --- ENABLE USER INPUT ---
    lcall Check_Buttons ; <--- New to poll user
    lcall Check_Keypad
    ; ----------------------------
	jbc PB1_flag, Control_FSM_state1_b
	sjmp Control_FSM_done
Control_FSM_state1_b:
	lcall Update_FSM_Variables ; <-- Added line for new function 	
	sjmp Control_FSM_state2_a ;<-- got rid of a line or else would be stuck in a loop

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
;         Main program.          
;-------------------------------------------------------------------------------;
main:
	; Initialization
    mov SP, #0x7F

; --- PORT CONFIGURATION (Changed bc old config. didn't have correct button inputs) ---
    ; P0: Odd=LCD(Out), Even=Buttons(In) 
    ; Binary: 10101010 -> Hex: 0xAA
    mov P0MOD, #0xAA 

    ; P1: Mixed usage 
    ; P1.7(LCD_RS), P1.6(Row3), P1.5(Sound), P1.4(Row2)
    ; P1.3(PWM), P1.2(Row1), P1.1(LCD_E) -> All Outputs
    ; P1.0 (Unused/RX) -> Input
    ; Binary: 11111110 -> Hex: 0xFE
    mov P1MOD, #0xFE

    ; P2: Row4(Out), Cols(In)
    ; P2.0 (Row4) is Out (1). P2.2, P2.4, P2.6 (Cols) are In (0).
    ; Binary: 00000001 -> Hex: 0x01
    mov P2MOD, #0x01

    ; P3: Col4(In)
    ; P3.0 (Col4) is In (0).
    mov P3MOD, #0x00
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
	clr state_change_signal

	; Set bit
	setb tc_startup_window

	lcall Timer0_Init
    lcall Timer2_Init
	lcall ELCD_4BIT
	;----- Two new lines I added to initialize the UI
	lcall Init_All_Buffers
    lcall Update_Screen_Full
	;-----
	lcall Initialize_Serial_Port
;-------------------------------------------------------------------------------;
; while(1) loop
;-------------------------------------------------------------------------------;
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
;-------------------------------------------------------------------------------;

; ================================================================
; UI & HELPER SUBROUTINES
; ================================================================

; ----------------------------------------------------------------
; MODULE: BRIDGE (Text to Integer Conversion)
; ----------------------------------------------------------------
Update_FSM_Variables:
    ; --- 1. SOAK TEMP ---
    mov R0, #Buf_Soak_Temp
    lcall Parse_Temp_String
    mov soak_temp+0, R7
    mov soak_temp+1, #0
    mov soak_temp+2, #0
    mov soak_temp+3, #0

    ; --- 2. REFLOW TEMP ---
    mov R0, #Buf_Refl_Temp
    lcall Parse_Temp_String
    mov reflow_temp+0, R7
    mov reflow_temp+1, #0
    mov reflow_temp+2, #0
    mov reflow_temp+3, #0

    ; --- 3. SOAK TIME ---
    mov R0, #Buf_Soak_Time
    lcall Parse_Time_String
    mov soak_time+0, R7
    mov soak_time+1, R6
    mov soak_time+2, #0
    mov soak_time+3, #0

    ; --- 4. REFLOW TIME ---
    mov R0, #Buf_Refl_Time
    lcall Parse_Time_String
    mov reflow_time+0, R7
    mov reflow_time+1, R6
    mov reflow_time+2, #0
    mov reflow_time+3, #0
    ret

; --- Helper: Parse "123" to Integer ---
Parse_Temp_String:
    mov R7, #0              ; Clear Result
Parse_Temp_Loop:
    mov A, @R0
    jz Parse_Temp_Done      ; If Null, we are done
    
    ; Convert ASCII to Digit
    clr C
    subb A, #0x30
    mov R5, A               ; R5 = New Digit
    
    ; Result = (Result * 10) + New Digit
    mov A, R7
    mov B, #10
    mul AB
    add A, R5
    mov R7, A
    
    inc R0
    sjmp Parse_Temp_Loop
Parse_Temp_Done:
    ret

; --- Helper: Parse "MMSS" to Seconds ---
Parse_Time_String:
    ; 1. Minutes Tens
    mov A, @R0
    subb A, #0x30
    mov B, #10
    mul AB
    mov R5, A
    inc R0
    
    ; 2. Minutes Ones
    mov A, @R0
    subb A, #0x30
    add A, R5
    mov R5, A               ; R5 = Total Minutes
    inc R0
    
    ; 3. Seconds Tens
    mov A, @R0
    subb A, #0x30
    mov B, #10
    mul AB
    mov R4, A
    inc R0
    
    ; 4. Seconds Ones
    mov A, @R0
    subb A, #0x30
    add A, R4               ; R4 = Total Seconds
    
    ; 5. Calculate Total Seconds = (Mins * 60) + Secs
    mov A, R5
    mov B, #60
    mul AB
    add A, R4
    mov R7, A               ; Low Byte
    mov A, B
    addc A, #0
    mov R6, A               ; High Byte
    ret

; ----------------------------------------------------------------
; MODULE: BUTTON HANDLER (Mode Selection)
; ----------------------------------------------------------------
Check_Buttons:
    jnb BTN_SOAK_TEMP, Btn_Soak_Temp_Press
    jnb BTN_SOAK_TIME, Btn_Soak_Time_Press
    jnb BTN_REFL_TEMP, Btn_Refl_Temp_Press
    jnb BTN_REFL_TIME, Btn_Refl_Time_Press
    ret

Btn_Soak_Temp_Press:
    lcall Wait_25ms
    mov Current_State, #1
    mov Cursor_Idx, #0
    sjmp Redraw_Screen

Btn_Soak_Time_Press:
    lcall Wait_25ms
    mov Current_State, #2
    mov Cursor_Idx, #0
    sjmp Redraw_Screen

Btn_Refl_Temp_Press:
    lcall Wait_25ms
    mov Current_State, #3
    mov Cursor_Idx, #0
    sjmp Redraw_Screen

Btn_Refl_Time_Press:
    lcall Wait_25ms
    mov Current_State, #4
    mov Cursor_Idx, #0
    sjmp Redraw_Screen

Redraw_Screen:
    lcall Update_Screen_Full
    ; Wait for button release
    jnb BTN_SOAK_TEMP, $
    jnb BTN_SOAK_TIME, $
    jnb BTN_REFL_TEMP, $
    jnb BTN_REFL_TIME, $
    ret

; ----------------------------------------------------------------
; MODULE: KEYPAD HANDLER (Input Logic)
; ----------------------------------------------------------------
Check_Keypad:
    ; If State is 0 (Home), ignore keypad
    mov A, Current_State
    jz Keypad_Exit
    
    lcall Keypad_Scan
    jnc Keypad_Exit         ; Carry = 0 means no key pressed

    ; --- Check Special Keys ---
    mov A, R7
    cjne A, #14, Check_Hash ; 14 is Star (*)
    
    ; Star Key Pressed: Reset Buffer
    lcall Reset_Current_Buffer
    lcall Update_Screen_Full
    mov Cursor_Idx, #0
    ret

Check_Hash:
    mov A, R7
    cjne A, #12, Check_Numeric ; 12 is Hash (#)
    ret                     ; Ignore Hash key

Check_Numeric:
    ; Ensure key is 0-9
    mov A, R7
    clr C
    subb A, #10
    jnc Symbol_Key_Ignored
    
    ; Convert to ASCII
    mov A, R7
    add A, #0x30
    mov R5, A

    ; Save to Buffer
    lcall Get_Current_Buffer_Addr
    mov A, Cursor_Idx
    add A, R0
    mov R0, A
    mov A, R5
    mov @R0, A
    inc Cursor_Idx

    ; --- Check Cursor Limits ---
    mov A, Current_State
    cjne A, #1, Check_Limit_Time_1
    sjmp Limit_Temp_3

Check_Limit_Time_1:
    cjne A, #3, Limit_Time_4
    sjmp Limit_Temp_3

Limit_Temp_3:
    mov A, Cursor_Idx
    cjne A, #3, Do_Refresh
    dec Cursor_Idx          ; Stay at last digit
    sjmp Do_Refresh

Limit_Time_4:
    mov A, Cursor_Idx
    cjne A, #4, Do_Refresh
    dec Cursor_Idx          ; Stay at last digit
    sjmp Do_Refresh

Do_Refresh:
    lcall Update_Screen_Full
    ret

Symbol_Key_Ignored:
    ret
Keypad_Exit:
    ret

; ----------------------------------------------------------------
; MODULE: HARDWARE SCANNER (Matrix Logic)
; ----------------------------------------------------------------
Keypad_Scan:
    ; Step 1: Check if ANY key is pressed (All Rows Low)
    clr ROW1
    clr ROW2
    clr ROW3
    clr ROW4
    mov C, COL1
    anl C, COL2
    anl C, COL3
    anl C, COL4
    jnc Keypad_Debounce
    clr C
    ret

Keypad_Debounce:
    lcall Wait_25ms
    mov C, COL1
    anl C, COL2
    anl C, COL3
    anl C, COL4
    jnc Keypad_Find_Row
    clr C
    ret

Keypad_Find_Row:
    setb ROW1
    setb ROW2
    setb ROW3
    setb ROW4

    ; Row 1
    clr ROW1
    jnb COL1, Keypad_Key_1
    jnb COL2, Keypad_Key_2
    jnb COL3, Keypad_Key_3
    jnb COL4, Keypad_Key_A
    setb ROW1

    ; Row 2
    clr ROW2
    jnb COL1, Keypad_Key_4
    jnb COL2, Keypad_Key_5
    jnb COL3, Keypad_Key_6
    jnb COL4, Keypad_Key_B
    setb ROW2

    ; Row 3
    clr ROW3
    jnb COL1, Keypad_Key_7
    jnb COL2, Keypad_Key_8
    jnb COL3, Keypad_Key_9
    jnb COL4, Keypad_Key_C
    setb ROW3

    ; Row 4
    clr ROW4
    jnb COL1, Keypad_Key_Star
    jnb COL2, Keypad_Key_0
    jnb COL3, Keypad_Key_Hash
    jnb COL4, Keypad_Key_D
    setb ROW4
    clr C
    ret

; Key Mapping (Renamed to avoid conflicts)
Keypad_Key_1: mov R7, #1
       sjmp Wait_Release
Keypad_Key_2: mov R7, #2
       sjmp Wait_Release
Keypad_Key_3: mov R7, #3
       sjmp Wait_Release
Keypad_Key_A: mov R7, #10
       sjmp Wait_Release
Keypad_Key_4: mov R7, #4
       sjmp Wait_Release
Keypad_Key_5: mov R7, #5
       sjmp Wait_Release
Keypad_Key_6: mov R7, #6
       sjmp Wait_Release
Keypad_Key_B: mov R7, #11
       sjmp Wait_Release
Keypad_Key_7: mov R7, #7
       sjmp Wait_Release
Keypad_Key_8: mov R7, #8
       sjmp Wait_Release
Keypad_Key_9: mov R7, #9
       sjmp Wait_Release
Keypad_Key_C: mov R7, #13
       sjmp Wait_Release
Keypad_Key_Star: mov R7, #14
       sjmp Wait_Release
Keypad_Key_0: mov R7, #0
       sjmp Wait_Release
Keypad_Key_Hash: mov R7, #12
       sjmp Wait_Release
Keypad_Key_D: mov R7, #15
       sjmp Wait_Release

Wait_Release:
    mov C, COL1
    anl C, COL2
    anl C, COL3
    anl C, COL4
    jnc Wait_Release
    setb C
    setb ROW1
    setb ROW2
    setb ROW3
    setb ROW4
    ret

Wait_25ms:
    mov R0, #15
W25_L3: mov R1, #74
W25_L2: mov R2, #250
W25_L1: djnz R2, W25_L1
    djnz R1, W25_L2
    djnz R0, W25_L3
    ret

; ----------------------------------------------------------------
; MODULE: BUFFER INIT (Reset Logic)
; ----------------------------------------------------------------
Init_All_Buffers:
    mov R0, #Buf_Soak_Temp
    lcall Init_Temp_Template
    mov R0, #Buf_Refl_Temp
    lcall Init_Temp_Template
    mov R0, #Buf_Soak_Time
    lcall Init_Time_Template
    mov R0, #Buf_Refl_Time
    lcall Init_Time_Template
    ret

Init_Temp_Template:
    mov @R0, #'0'
    inc R0
    mov @R0, #'0'
    inc R0
    mov @R0, #'0'
    inc R0
    mov @R0, #0
    ret

Init_Time_Template:
    mov @R0, #'0'
    inc R0
    mov @R0, #'0'
    inc R0
    mov @R0, #'0'
    inc R0
    mov @R0, #'0'
    inc R0
    mov @R0, #0
    ret

Reset_Current_Buffer:
    mov A, Current_State
    cjne A, #1, Reset_Chk_2
    mov R0, #Buf_Soak_Temp
    lcall Init_Temp_Template
    ret
Reset_Chk_2:
    cjne A, #2, Reset_Chk_3
    mov R0, #Buf_Soak_Time
    lcall Init_Time_Template
    ret
Reset_Chk_3:
    cjne A, #3, Reset_Chk_4
    mov R0, #Buf_Refl_Temp
    lcall Init_Temp_Template
    ret
Reset_Chk_4:
    mov R0, #Buf_Refl_Time
    lcall Init_Time_Template
    ret

; ----------------------------------------------------------------
; MODULE: SCREEN UPDATE (Visual Logic)
; ----------------------------------------------------------------
Update_Screen_Full:
    lcall Clear_Screen_Func
    Set_Cursor(1, 1)

    ; --- Draw Line 1 (Titles) ---
    mov A, Current_State
    cjne A, #0, Update_State_1
    Send_Constant_String(#Txt_Home)
    ret 
Update_State_1:
    cjne A, #1, Update_State_2
    Send_Constant_String(#Txt_SoakT)
    sjmp Draw_Temp_Format
Update_State_2:
    cjne A, #2, Update_State_3
    Send_Constant_String(#Txt_SoakTime)
    sjmp Draw_Time_Format
Update_State_3:
    cjne A, #3, Update_State_4
    Send_Constant_String(#Txt_ReflT)
    sjmp Draw_Temp_Format
Update_State_4:
    Send_Constant_String(#Txt_ReflTime)
    sjmp Draw_Time_Format

; --- Draw Line 2 (Values) ---
Draw_Temp_Format:
    Set_Cursor(2, 1)
    lcall Get_Current_Buffer_Addr
    lcall Print_String_RAM
    mov A, #'C'
    lcall ?WriteData
    sjmp Restore_Cursor

Draw_Time_Format:
    Set_Cursor(2, 1)
    lcall Get_Current_Buffer_Addr
    ; MM
    mov A, @R0
    lcall ?WriteData
    inc R0
    mov A, @R0
    lcall ?WriteData
    inc R0
    ; Colon
    mov A, #':'
    lcall ?WriteData
    ; SS
    mov A, @R0
    lcall ?WriteData
    inc R0
    mov A, @R0
    lcall ?WriteData
    ; Unit
    mov A, #'s'
    lcall ?WriteData
    sjmp Restore_Cursor

; --- Restore Cursor Position ---
Restore_Cursor:
    mov A, Current_State
    cjne A, #2, Check_State_4
    sjmp Adjust_Cursor_Time
Check_State_4:
    cjne A, #4, Normal_Cursor
    sjmp Adjust_Cursor_Time

Normal_Cursor:
    mov A, Cursor_Idx
    add A, #0xC0
    lcall ?WriteCommand
    ret

Adjust_Cursor_Time:
    ; Skip the colon index (2)
    mov A, Cursor_Idx
    cjne A, #2, No_Skip
    inc A 
No_Skip:
    ; Add 1 if past the colon
    clr C
    subb A, #2
    jc No_Add
    mov A, Cursor_Idx
    inc A
    sjmp Final_Cursor_Set
No_Add:
    mov A, Cursor_Idx
Final_Cursor_Set:
    add A, #0xC0
    lcall ?WriteCommand
    ret

Print_String_RAM:
    mov A, @R0
    jz Print_String_Done
    lcall ?WriteData
    inc R0
    sjmp Print_String_RAM
Print_String_Done:
    ret

Clear_Screen_Func:
    mov A, #0x01
    lcall ?WriteCommand
    mov R2, #10
    lcall Wait_25ms
    mov A, #0x0F
    lcall ?WriteCommand
    ret

Get_Current_Buffer_Addr:
    mov A, Current_State
    cjne A, #1, Get_Buf_2
    mov R0, #Buf_Soak_Temp
    ret
Get_Buf_2:
    cjne A, #2, Get_Buf_3
    mov R0, #Buf_Soak_Time
    ret
Get_Buf_3:
    cjne A, #3, Get_Buf_4
    mov R0, #Buf_Refl_Temp
    ret
Get_Buf_4:
    mov R0, #Buf_Refl_Time
    ret
    
; --- Helper to prevent "Machine Gun" button presses ---
Wait_For_P1_0_Release:
    jnb P1.0, $    ; Wait here while the button is still pressed (0)
    ret

END
