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
;ADC_C DATA 0xa1
;ADC_L DATA 0xa2
;ADC_H DATA 0xa3
$include(..\inc\math32.asm) ; 
$LIST
; ----------------------------------------------------------------------------------------------;
; Data Segment 0x30 -- 0x7F  (overall 79d bytes available)
dseg at 0x30
; time buffer 
current_time_sec:     ds 1
current_time_minute:  ds 1
soak_time_sec:        ds 1
soak_time_minute:     ds 1
reflow_time_sec:      ds 1
reflow_time_minute:   ds 1
soak_end_time_sec:      ds 1
soak_end_time_minute:   ds 1
reflow_end_time_sec:    ds 1
reflow_end_time_minute: ds 1

; math32 buffer variables
x:      ds  4
y:      ds  4
bcd:    ds  5

current_temp: ds 4 ;
soak_temp:    ds 4 ;
reflow_temp:  ds 4 ;

wait25_btn_cnt:    ds 1
wait25_keypad_cnt: ds 1
wait25_adc_cnt:    ds 1
wait25_lcd_cnt:    ds 1
wait25_count:      ds 1

power_output:  ds 4 ;
pwm_counter:   ds 4 ; counter for pwm (0-1500_timer: ds 1

Control_FSM_state: ds 1 
Current_State:     ds 1

soak_temp_diff: ds 4 ; temperature difference between target soak temp and current oven temp 
proportional_gain_var: ds 4 ; power gain calculated from the proportional block

;-- UI buffers I added (ayaan)
Cursor_Idx: ds 1

; Buzzer module variables
buzz_state:      ds 1   ; 0=IDLE, 1=ON, 2=OFF
buzz_timer:      ds 1   ; counts ms within ON/OFF window
buzz_beeps_left: ds 1   ; how many beeps remaining
buzz_priority:   ds 1   ; 0 none, 1=state, 2=done, 3=error

SEC_FSM_timer: ds 1
SEC_FSM_state: ds 1
; Push buttons for globel interrupt 
PB0_DEB_timer:  ds 1
PB0_DEB_state:  ds 1
PB2_DEB_timer:  ds 1
PB2_DEB_state:  ds 1

; Buzzer state
beep_count:  ds 1      ; remaining beeps
beep_state:  ds 1      ; 0=idle, 1=ON, 2=OFF
beep_tmr:    ds 2      ; 16-bit ms timer (needs to reach 500)

servo_pwm_counter: ds 1 ; counter for the servo pwm signal
; In your data section
BTN_DEB_state: ds 1
BTN_DEB_timer: ds 1
BTN_DEB_id:    ds 1

; UART RX state (polling)
rx_idx:    ds 1
rx_ready:  ds 1
; 79

iseg at 0x80
Buf_Soak_Temp: ds 4   
Buf_Soak_Time: ds 5   
Buf_Refl_Temp: ds 4   
Buf_Refl_Time: ds 5

; UART RX line buffer (polling) in upper RAM
rx_buf:        ds 40    ; null-terminated command line
; 
;-------------------------------------------------------------------------------
; bit operation setb, clr, jb, and jnb
bseg
mf:     dbit 1 ; math32 sign
one_second_flag: dbit 1
one_ms_pwm_flag: dbit 1 ; one_millisecond_flag for pwm signal
one_ms_buzz_flag: dbit 1 ; one_millisecond_flag for buzz
one_second_lcd_flag: dbit 1

soak_temp_reached: dbit 1
reflow_temp_reached: dbit 1
cooling_temp_reached: dbit 1

soak_time_reached: dbit 1
reflow_time_reached: dbit 1

reset_signal: dbit 1
stop_signal: dbit 1
start_signal_count: dbit 1
time_count_doing_signal: dbit 1
config_finish_signal: dbit 1

state_change_signal: dbit 1
state_change_signal_TC: dbit 1
state_change_signal_Count: dbit 1
state_change_beep_signal: dbit 1

tc_missing_abort: dbit 1   ; 1 = abort because temp < 50C after 60s
tc_startup_window: dbit 1   ; 1 = still within first 60 seconds of the run

PB0_flag: dbit 1 ; start entire program
PB1_flag: dbit 1 ; start soak
PB2_flag: dbit 1 ; pause process

;buzzer beep
one_ms_beep_flag: dbit 1
beep_error_done: dbit 1

; BSEG (Bit Segment)
wait25_active: dbit 1 ; 1 = We are currently waiting
wait25_done:   dbit 1 ; 1 = The 25ms has finished
wait25_btn_active:    dbit 1
wait25_btn_done:      dbit 1
wait25_keypad_active: dbit 1
wait25_keypad_done:   dbit 1
wait25_adc_active:    dbit 1
wait25_adc_done:      dbit 1
wait25_lcd_active:    dbit 1
wait25_lcd_done:      dbit 1

fullscreen_update_signal: dbit 1

one_second_flag_test: dbit 1
one_millisecond_flag_servo: dbit 1 ; set the one millsiecond flag for servo pwm signal generation

servo_angle_zero: dbit 1 ; flag for indicating whether the servo angle should be at 0 or not: 1 -> 0; 0 -> 180
soak_temp_greater: dbit 1 ; target soak_temp greater than current_temp

remote_config_mode: dbit 1
; 40 bits used

;-------------------------------------------------------------------------------
cseg
CLK            EQU 33333333 ; Microcontroller system crystal frequency in Hz
BAUD           EQU 57600

TIMER0_RATE    EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD  EQU ((65536-(CLK/(12*TIMER0_RATE)))) ; The prescaler in the CV-8052 
; is always 12 unlike the N76E003 where is selectable.

TIMER_1_RELOAD EQU (256-((2*CLK)/(12*32*BAUD)))

TIMER2_RATE    EQU 1000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD  EQU ((65536-(CLK/(12*TIMER2_RATE))))

PWM_PERIOD     EQU 1499 ; 1.5s period

SOUND_OUT      EQU P1.5 ; Pin connected to the speaker
BEEP_ON_MS	   EQU 100  ; 100ms
BEEP_OFF_MS    EQU 100  ; 100ms

PWM_OUT        EQU P1.3 ; Pin connected to the ssr for outputing pwm signal

; These 'equ' must match the wiring between the DE10Lite board and the LCD
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
PB0 		  equ P1.0
PB2 		  equ P3.7

; --- PB0PAD ---
ROW1 equ P1.2
ROW2 equ P1.4
ROW3 equ P1.6
ROW4 equ P2.0
COL1 equ P2.2
COL2 equ P2.4
COL3 equ P2.6
COL4 equ P3.0

DC_OUT         EQU P4.0
SERVO_OUT      EQU p3.6 ; servo pin
LED_LEFT       EQU P3.4 ; left LED (PB3.4)
LED_MID        EQU P3.3 ; middle LED (PB3.3)
LED_RIGHT      EQU P3.2 ; right LED (PB3.2)

SERVO_PERIOD   EQU 20 ; pwm signal period for the servo motor (20 ms)
SERVO_0        EQU 1 ; pwm high time for the servo motor to stay at 0 degree
SERVO_180      EQU 2 ; pwm high time for the servo motor to stay at 180 degrees

COLD_JUNCTION_TEMP equ 20
MAX_POWER	   EQU 1500 ; max oven power
NO_POWER	   EQU 0    ; no power
BASE_POWER     EQU (MAX_POWER/5) ; 20% base power for state 2, 4
HALF_POWER     EQU (MAX_POWER/2) ; 50% power indicator
KP			   EQU 5 ; proportional gain

;1234567890123456 <-- 16 characters per line LCD
Initial_Message:  db 'initial message', 0
String_state0_1:  db 'Welcome        ', 0
String_state0_2:  db 'Press PB0      ', 0

; --- UI STRINGS (REQUIRED FOR PB0PAD LOGIC), <- I can fix if duplicates
Txt_Home:     db 'Select Mode:    ', 0
Txt_SoakT:    db 'Set Soak Temp   ', 0
Txt_SoakTime: db 'Set Soak Time   ', 0
Txt_ReflT:    db 'Set Reflow Temp ', 0
Txt_ReflTime: db 'Set Reflow Time ', 0

; 1234567890123456
String_state1:      db 'Set Parameters ', 0
String_soak_temp:   db 'Soak Temp:', 0
String_reflow_temp: db 'Reflow Temp:', 0
String_soak_time:   db 'Soak Time:', 0
String_reflow_time: db 'Reflow Time:', 0

String_temp_line:  db 'Temp: ', 0

; 1234567890123456
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
;   Timer 1: Serial port baud rate 57600 generator
;   Timer 2: 1ms interrupt for BCD counter increment/decrement
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
    ; setb TR0  (no need to open at first)
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
    jbc TI, putchar_L1
    sjmp putchar
putchar_L1:
    mov SBUF,a
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

;------------------------------------------------------------
; getchar_nb (non-blocking)
; OUT: C=1 if got byte, A=byte
;      C=0 if none
;------------------------------------------------------------
getchar_nb:
    jnb RI, rx_none
    mov A, SBUF
    clr RI
    setb C
    ret
rx_none:
    clr C
    ret
;------------------------------------------------------------
; Serial_RX_Pump
; Builds a null-terminated line in rx_buf.
; Sets rx_ready=1 when a full line received.
;------------------------------------------------------------
Serial_RX_Pump:
    mov A, rx_ready
    jnz rxp_done          ; don't overwrite unprocessed line

rxp_more:
    lcall getchar_nb
    jnc rxp_done          ; no new byte
    mov B, A              ; save received byte

    ; ignore CR
    cjne A, #0DH, rxp_not_cr
    sjmp rxp_more

rxp_not_cr:
    ; if LF -> finish line
    cjne A, #0AH, rxp_store

    ; terminate string
    mov A, rx_idx
    add A, #rx_buf
    mov R0, A
    mov @R0, #0
    mov rx_ready, #1
    mov rx_idx, #0
    sjmp rxp_done

rxp_store:
    ; store char if room (max 39 chars)
    mov A, rx_idx
    cjne A, #39, rxp_ok
    mov rx_idx, #0        ; overflow: reset
    sjmp rxp_done

rxp_ok:
    mov A, rx_idx
    add A, #rx_buf
    mov R0, A
    mov A, B
    mov @R0, A
    inc rx_idx
    sjmp rxp_more

rxp_done:
    ret
; copies 3 ASCII digits to buffer at R1, null terminates
; R0 = src (first digit), R1 = dst
Copy3DigitsToBuf:
    mov A, @R0
    mov @R1, A
    inc R0
    inc R1
    mov A, @R0
    mov @R1, A
    inc R0
    inc R1
    mov A, @R0
    mov @R1, A
    inc R1
    mov @R1, #0
    ret

; copies 4 ASCII digits to buffer at R1, null terminates
Copy4DigitsToBuf:
    mov A, @R0
    mov @R1, A
    inc R0
    inc R1
    mov A, @R0
    mov @R1, A
    inc R0
    inc R1
    mov A, @R0
    mov @R1, A
    inc R0
    inc R1
    mov A, @R0
    mov @R1, A
    inc R1
    mov @R1, #0
    ret

;------------------------------------------------------------
; Serial_Process_Line
; Handles: UI:REMOTE, RUN:0/1, S:TTT, K:MMSS, R:TTT, L:MMSS, 
;          CFG:APPLY, CFG {json}, SAVE:1
;------------------------------------------------------------
Serial_Process_Line:
    mov A, rx_ready
    jnz SPL_HAVE
    ret
SPL_HAVE:
    mov rx_ready, #0
    mov R0, #rx_buf
    mov A, @R0

    ; Branch by first character (with trampolines for distance)
    cjne A, #'U', SPL_not_U
    sjmp do_chk_UI_REMOTE
SPL_not_U:
    cjne A, #'R', SPL_not_R
    sjmp do_chk_R_commands
SPL_not_R:
    cjne A, #'S', SPL_not_S
    sjmp do_chk_S_commands
SPL_not_S:
    cjne A, #'K', SPL_not_K
    sjmp do_chk_K
SPL_not_K:
    cjne A, #'L', SPL_not_L
    sjmp do_chk_L
SPL_not_L:
    cjne A, #'C', SPL_not_C
    sjmp do_chk_CFG_commands
SPL_not_C:
    ljmp spl_done

; --- Trampolines ---
do_chk_UI_REMOTE:
    ljmp chk_UI_REMOTE
do_chk_R_commands:
    ljmp chk_R_commands
do_chk_S_commands:
    ljmp chk_S_commands
do_chk_K:
    ljmp chk_K
do_chk_L:
    ljmp chk_L
do_chk_CFG_commands:
    ljmp chk_CFG_commands

;------------------------------------------------------------
; UI:REMOTE - Switch to remote control mode
;------------------------------------------------------------
chk_UI_REMOTE:
    mov R0, #rx_buf
    mov A, @R0
    cjne A, #'U', spl_done_bridge1
    inc R0
    mov A, @R0
    cjne A, #'I', spl_done_bridge1
    inc R0
    mov A, @R0
    cjne A, #':', spl_done_bridge1
    inc R0
    mov A, @R0
    cjne A, #'R', spl_done_bridge1
    ; Good enough - it's UI:R...
    sjmp spl_done_bridge1

spl_done_bridge1:
    ljmp spl_done

;------------------------------------------------------------
; R commands: R:TTT (reflow temp) or RUN:0/1
;------------------------------------------------------------
chk_R_commands:
    mov R0, #rx_buf
    inc R0                     ; Point to second char
    mov A, @R0
    cjne A, #':', chk_RUN_jump ; If not ':', check for RUN
    ; It's "R:" - Reflow temperature
    inc R0
    mov R1, #Buf_Refl_Temp
    lcall Copy3DigitsToBuf
    sjmp spl_done_bridge2

chk_RUN_jump:
    sjmp chk_RUN

spl_done_bridge2:
    ljmp spl_done

chk_RUN:
    ; Check for "RUN:"
    mov R0, #rx_buf
    inc R0
    mov A, @R0
    cjne A, #'U', spl_done_bridge3
    inc R0
    mov A, @R0
    cjne A, #'N', spl_done_bridge3
    inc R0
    mov A, @R0
    cjne A, #':', spl_done_bridge3
    inc R0
    mov A, @R0
    
    ; Check for '1' or '0'
    cjne A, #'1', chk_RUN_zero
    
    ; RUN:1 - Start the process
    ljmp Control_FSM_state2_a
    sjmp spl_done_bridge3

chk_RUN_zero:
    cjne A, #'0', spl_done_bridge3
    ; RUN:0 - Stop the process
    ljmp Control_FSM_state0_a
    sjmp spl_done_bridge3

spl_done_bridge3:
    ljmp spl_done

;------------------------------------------------------------
; S commands: S:TTT (soak temp) or SAVE:1
;------------------------------------------------------------
chk_S_commands:
    mov R0, #rx_buf
    inc R0                     ; Point to second char
    mov A, @R0
    cjne A, #':', chk_SAVE_jump
    ; It's "S:" - Soak temperature
    inc R0
    mov R1, #Buf_Soak_Temp
    lcall Copy3DigitsToBuf
    sjmp spl_done_bridge4

chk_SAVE_jump:
    sjmp chk_SAVE

spl_done_bridge4:
    ljmp spl_done

chk_SAVE:
    ; Check for "SAVE:"
    mov R0, #rx_buf
    inc R0
    mov A, @R0
    cjne A, #'A', spl_done_bridge5
    inc R0
    mov A, @R0
    cjne A, #'V', spl_done_bridge5
    inc R0
    mov A, @R0
    cjne A, #'E', spl_done_bridge5
    inc R0
    mov A, @R0
    cjne A, #':', spl_done_bridge5
    inc R0
    mov A, @R0
    cjne A, #'1', spl_done_bridge5
    
    ; SAVE:1 - Save to non-volatile memory (stub)
    sjmp spl_done_bridge5

spl_done_bridge5:
    ljmp spl_done

;------------------------------------------------------------
; K:MMSS - Soak time
;------------------------------------------------------------
chk_K:
    mov R0, #rx_buf
    inc R0
    mov A, @R0
    cjne A, #':', spl_done_bridge6
    inc R0
    mov R1, #Buf_Soak_Time
    lcall Copy4DigitsToBuf
    sjmp spl_done_bridge6

spl_done_bridge6:
    ljmp spl_done

;------------------------------------------------------------
; L:MMSS - Reflow time
;------------------------------------------------------------
chk_L:
    mov R0, #rx_buf
    inc R0
    mov A, @R0
    cjne A, #':', spl_done_bridge7
    inc R0
    mov R1, #Buf_Refl_Time
    lcall Copy4DigitsToBuf
    sjmp spl_done_bridge7

spl_done_bridge7:
    ljmp spl_done

;------------------------------------------------------------
; CFG commands: CFG:APPLY or CFG {json}
;------------------------------------------------------------
chk_CFG_commands:
    mov R0, #rx_buf
    mov A, @R0
    cjne A, #'C', spl_done_bridge8
    inc R0
    mov A, @R0
    cjne A, #'F', spl_done_bridge8
    inc R0
    mov A, @R0
    cjne A, #'G', spl_done_bridge8
    inc R0
    mov A, @R0
    
    ; Check if ':' (CFG:APPLY) or ' ' (CFG {json})
    cjne A, #':', chk_CFG_json
    
    ; It's "CFG:" - check for APPLY
    inc R0
    mov A, @R0
    cjne A, #'A', spl_done_bridge8
    inc R0
    mov A, @R0
    cjne A, #'P', spl_done_bridge8
    inc R0
    mov A, @R0
    cjne A, #'P', spl_done_bridge8
    inc R0
    mov A, @R0
    cjne A, #'L', spl_done_bridge8
    inc R0
    mov A, @R0
    cjne A, #'Y', spl_done_bridge8
    
    ; CFG:APPLY - Apply configuration
    lcall Update_FSM_Variables
    setb state_change_signal
    setb fullscreen_update_signal
    sjmp spl_done_bridge8

chk_CFG_json:
    ; Check for space (CFG {json})
    cjne A, #' ', spl_done_bridge8
    ; It's "CFG {...}" - JSON config (ignored)
    sjmp spl_done_bridge8

spl_done_bridge8:
    ljmp spl_done

;------------------------------------------------------------
spl_done:
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
; Serial temperature line for PuTTY/screen
; Outputs: "Temp: XXXC\r\n"
;-------------------------------------------------------------------------------
Serial_Send_Temp_Line:
    mov dptr, #String_temp_line
    lcall SendString

    ; Convert current_temp to BCD (same as LCD)
    mov x, current_temp
    mov x+1, current_temp+1
    mov x+2, current_temp+2
    mov x+3, current_temp+3
    lcall hex2bcd

    mov R7, #0          ; printed_flag = 0

    ; Print Hundreds (if non-zero)
    mov a, bcd+1
    anl a, #0x0F
    jz Serial_Skip_Hundreds
    add a, #0x30
    lcall putchar
    mov R7, #1
Serial_Skip_Hundreds:

    ; Print Tens (if non-zero or if hundreds already printed)
    mov a, bcd+0
    swap a
    anl a, #0x0F
    jnz Serial_Print_Tens
    mov a, R7
    jz Serial_Skip_Tens
Serial_Print_Tens:
    mov a, bcd+0
    swap a
    anl a, #0x0F
    add a, #0x30
    lcall putchar
    mov R7, #1
Serial_Skip_Tens:

    ; Print Ones (always)
    mov a, bcd+0
    anl a, #0x0F
    add a, #0x30
    lcall putchar

    ; Print 'C' and newline
    mov a, #'C'
    lcall putchar
    mov a, #0DH     ; CR
    lcall putchar
    mov a, #0AH     ; LF
    lcall putchar
    ret
;-------------------------------------------------------------------------------
$include(..\inc\Timer2_ISR.inc) ; Timer 2 ISR for 1ms tick and pwm signal generation
$include(..\inc\LCD_4bit_DE10Lite_no_RW.inc) ; LCD related functions and utility macros
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
LCD_Print_2Digits:
    lcall Hex_to_bcd_8bit
    mov a, R0
    swap a
    anl a, #0x0F
    add a, #0x30
    lcall ?WriteData
    mov a, R0
    anl a, #0x0F
    add a, #0x30
    lcall ?WriteData
    ret

LCD_Display_Update_func:
    push acc
    
    ; ==========================================
    ; PART 1: STATIC TEXT (Title)
    ; Runs ONLY when the state changes
    ; ==========================================
    
    ; [FIX] "Trampoline" logic for long distance jump
    ; If signal is SET (1), we stay here and update.
    ; If signal is CLEAR (0), we Long Jump to the Live Update section.
    jnb state_change_signal, Do_Dynamic_Update
	clr state_change_signal
	sjmp Do_Static_Update
Do_Dynamic_Update:
	ljmp Check_Live_Update

Do_Static_Update:
    ; State Changed: Clear Screen and Write Title
    lcall Clear_Screen_Func
    mov a, Control_FSM_state
    
    ; State 0: Welcome
    cjne a, #0, LCD_Check_1
    Set_Cursor(1,1)
    Send_Constant_String(#String_state0_1)
    Set_Cursor(2,1)
    Send_Constant_String(#String_state0_2)
    ljmp LCD_Done_Bridge ; Exit

LCD_Check_1: ; Setup
    cjne a, #1, LCD_Check_2
    Set_Cursor(1,1)
    Send_Constant_String(#String_state1)
    ljmp LCD_Done_Bridge

LCD_Check_2: ; Ramp to Soak
    cjne a, #2, LCD_Check_3
    Set_Cursor(1,1)
    Send_Constant_String(#String_state2)
    ljmp LCD_Update_Temp_Value

LCD_Check_3: ; Soak
    cjne a, #3, LCD_Check_4
    Set_Cursor(1,1)
    Send_Constant_String(#String_state3)
    ljmp LCD_Update_Temp_Value

LCD_Check_4: ; Ramp to Peak
    cjne a, #4, LCD_Check_5
    Set_Cursor(1,1)
    Send_Constant_String(#String_state4)
    ljmp LCD_Update_Temp_Value

LCD_Check_5: ; Reflow
    cjne a, #5, LCD_Check_6
    Set_Cursor(1,1)
    Send_Constant_String(#String_state5)
    ljmp LCD_Update_Temp_Value

LCD_Check_6: ; Cooling
    cjne a, #6, LCD_Check_7
    Set_Cursor(1,1)
    Send_Constant_String(#String_state6)
    ljmp LCD_Update_Temp_Value

LCD_Check_7: ; Done
    cjne a, #7, LCD_Done_Bridge ; If not 7, we are done
    Set_Cursor(1,1)
    Send_Constant_String(#String_state7)
    ljmp LCD_Done_Bridge

; Local bridge to reach the far-away LCD_Done
LCD_Done_Bridge:
    ljmp LCD_Done

;-------------------------------------------------------------------------------
; PART 2: dyanmic for temp
; runs every time 'one_second_flag' is set
;-------------------------------------------------------------------------------
Check_Live_Update:
    jnb one_second_flag, LCD_Done_Bridge
    clr one_second_flag
    
    ; Only update temp for States 2, 3, 4, 5, 6
    mov a, Control_FSM_state
    cjne a, #2, Check_St3
    sjmp LCD_Update_Temp_Value
Check_St3:
    cjne a, #3, Check_St4
    sjmp LCD_Update_Temp_Value
Check_St4:
    cjne a, #4, Check_St5
    sjmp LCD_Update_Temp_Value
Check_St5:
    cjne a, #5, Check_St6
    sjmp LCD_Update_Temp_Value
Check_St6:
    cjne a, #6, LCD_Done
    sjmp LCD_Update_Temp_Value

LCD_Update_Temp_Value:
    Set_Cursor(2, 1)
    mov x, current_temp
    mov x+1, current_temp+1
    mov x+2, current_temp+2
    mov x+3, current_temp+3
    lcall hex2bcd
    lcall Update_HEX_Temp 
    mov a, bcd+1
    anl a, #0x0F
    add a, #0x30
    lcall ?WriteData
    mov a, bcd+0
    swap a
    anl a, #0x0F
    add a, #0x30
    lcall ?WriteData
    mov a, bcd+0
    anl a, #0x0F
    add a, #0x30
    lcall ?WriteData
    mov a, #'C'
    lcall ?WriteData
    mov a, #' '
    lcall ?WriteData
    lcall ?WriteData
    Set_Cursor(2, 12)
    mov a, current_time_minute
    lcall LCD_Print_2Digits
    mov a, #':'
    lcall ?WriteData
    mov a, current_time_sec
    lcall LCD_Print_2Digits
    lcall Serial_Send_Temp_Line

LCD_Done:
    pop acc
    ret

; ----------------------------------------------------------------
; MODULE: SCREEN UPDATE (Visual Logic)
; ----------------------------------------------------------------
Update_Screen_Full:
	mov a, Control_FSM_state
	cjne a, #1, Update_Screen_Full_ret
	sjmp Update_Screen_Full_do
Update_Screen_Full_ret:
	ret
Update_Screen_Full_do:
    jnb fullscreen_update_signal, Update_Screen_Full_ret
    clr fullscreen_update_signal

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
    Display_char(#'C')
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
    Display_char(#':')
    ; SS
    mov A, @R0
    lcall ?WriteData
    inc R0
    mov A, @R0
    lcall ?WriteData
    ; Unit
    Display_char(#'s')
    sjmp Restore_Cursor

; --- Restore Cursor Position ---
Restore_Cursor:
    mov A, Current_State
    cjne A, #2, RC_Check_State_4  
    sjmp Adjust_Cursor_Time
RC_Check_State_4:             
    cjne A, #4, Normal_Cursor
    sjmp Adjust_Cursor_Time

Normal_Cursor:
    mov A, Cursor_Idx
    add A, #0xC0
    lcall ?WriteCommand

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

; --- Clear Screen with hardware delay ---
Clear_Screen_Func:
    WriteCommand(#0x01)        ; Clear display command
    Wait_Milli_Seconds(#2)     ; LCD needs ~2ms to clear
    WriteCommand(#0x0C)        ; Display ON, Cursor OFF
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

;-------------------------------------------------------------------------------
; Update HEX2-HEX0 with temperature (3 digits)
;-------------------------------------------------------------------------------
Update_HEX_Temp:
    mov dptr, #T_7seg
    ; Hundreds -> HEX2
    mov a, bcd+1
    anl a, #0x0F
    movc a, @a+dptr
    mov HEX2, a
    ; Tens -> HEX1
    mov a, bcd+0
    swap a
    anl a, #0x0F
    movc a, @a+dptr
    mov HEX1, a
    ; Ones -> HEX0
    mov a, bcd+0
    anl a, #0x0F
    movc a, @a+dptr
    mov HEX0, a
    ret
    
;---------------------------------------------------------
PB0_DEB:
;non-blocking state machine for PB0 debounce
    mov a, PB0_DEB_state
PB0_DEB_state0:
    cjne a, #0, PB0_DEB_state1
    jb PB0, PB0_DEB_done
    mov PB0_DEB_timer, #0
    inc PB0_DEB_state
    sjmp PB0_DEB_done
PB0_DEB_state1:
    cjne a, #1, PB0_DEB_state2
    ; this is the debounce state
    mov a, PB0_DEB_timer
    cjne a, #50, PB0_DEB_done ; 50 ms passed?
    inc PB0_DEB_state
    sjmp PB0_DEB_done  
PB0_DEB_state2:
    cjne a, #2, PB0_DEB_state3
    jb PB0, PB0_DEB_state2b
    inc PB0_DEB_state
    sjmp PB0_DEB_done  
PB0_DEB_state2b:
    mov PB0_DEB_state, #0
    sjmp PB0_DEB_done
PB0_DEB_state3:
    cjne a, #3, PB0_DEB_done
    jnb PB0, PB0_DEB_done
    setb PB0_flag ; Suscesfully detected a valid PB0 press/release
	cpl LEDRA.5
    mov PB0_DEB_state, #0  
PB0_DEB_done:
    ret

PB2_DEB:
;non-blocking state machine for PB2 debounce
    mov a, PB2_DEB_state
PB2_DEB_state0:
    cjne a, #0, PB2_DEB_state1
    jb PB2, PB2_DEB_done
    mov PB2_DEB_timer, #0
    inc PB2_DEB_state
    sjmp PB2_DEB_done
PB2_DEB_state1:
    cjne a, #1, PB2_DEB_state2
    ; this is the debounce state
    mov a, PB2_DEB_timer
    cjne a, #50, PB2_DEB_done ; 50 ms passed?
    inc PB2_DEB_state
    sjmp PB2_DEB_done  
PB2_DEB_state2:
    cjne a, #2, PB2_DEB_state3
    jb PB2, PB2_DEB_state2b
    inc PB2_DEB_state
    sjmp PB2_DEB_done  
PB2_DEB_state2b:
    mov PB2_DEB_state, #0
    sjmp PB2_DEB_done
PB2_DEB_state3:
    cjne a, #3, PB2_DEB_done
    jnb PB2, PB2_DEB_done
    setb PB2_flag ; Suscesfully detected a valid PB2 press/release
    mov PB2_DEB_state, #0  
PB2_DEB_done:
    ret

; ------------------------------------------------------------------------------
; Non-blocking FSM for the one second counter
;-------------------------------------------------------------------------------
SEC_FSM:
    mov a, SEC_FSM_state
SEC_FSM_state0:
    cjne a, #0, SEC_FSM_state1
    mov a, SEC_FSM_timer
    cjne a, #250, SEC_FSM_done
    mov SEC_FSM_timer, #0
    inc SEC_FSM_state
    sjmp SEC_FSM_done
SEC_FSM_state1:	
    cjne a, #1, SEC_FSM_state2
    setb LEDRA.1
    mov a, SEC_FSM_timer
    cjne a, #250, SEC_FSM_done
    mov SEC_FSM_timer, #0
    inc SEC_FSM_state
    sjmp SEC_FSM_done
SEC_FSM_state2:	
    cjne a, #2, SEC_FSM_state3
    setb LEDRA.2
    mov a, SEC_FSM_timer
    cjne a, #250, SEC_FSM_done
    mov SEC_FSM_timer, #0
    inc SEC_FSM_state
    sjmp SEC_FSM_done
SEC_FSM_state3:	
    cjne a, #3, SEC_FSM_done
    setb LEDRA.3
    mov a, SEC_FSM_timer
    cjne a, #250, SEC_FSM_done
    mov SEC_FSM_timer, #0
    mov SEC_FSM_state, #0
    
    ; These flags are always set (global use)
    setb one_second_lcd_flag
    setb one_second_flag
    
    ; Heartbeat LED always toggles
    cpl LEDRA.0
    
    ; Only update time if counting is enabled
    jnb time_count_doing_signal, SEC_FSM_done
    
    ; Update current time (only when counting)
    mov a, current_time_sec
    inc a
    cjne a, #60, SEC_NoMinuteCarry
    mov current_time_sec, #0
    inc current_time_minute
    sjmp SEC_FSM_done
SEC_NoMinuteCarry:
    mov current_time_sec, a
SEC_FSM_done:
    ret

; ------------------------------------------------------------------------------
; Counting the processing time 
;-------------------------------------------------------------------------------
Time_Counter:
    push ACC
    push psw
    mov a, Control_FSM_state
    
    ; State 2: Start counting
    cjne a, #2, Time_Counter_Nstate2
    jbc state_change_signal_Count, Time_Counter_Start
    sjmp Time_Counter_Done

Time_Counter_Start:
    mov current_time_sec, #0
    mov current_time_minute, #0
    setb time_count_doing_signal
    sjmp Time_Counter_Done

Time_Counter_Nstate2:
    ; State 6: Stop counting
    cjne a, #6, Time_Counter_Done
    clr time_count_doing_signal

Time_Counter_Done:
    pop psw
    pop ACC
    ret


;-------------------------------------------------------------------------------
; Time_Compare_MMSS
;
; PURPOSE:
;   Compare elapsed time (current_time_minute:current_time_sec)
;   against soak and reflow setpoints (soak_time_*, reflow_time_*).
;
; BEHAVIOR:
;   - If current_time >= soak_end_time   then soak_time_reached = 1
;   - If current_time >= reflow_end_time then reflow_time_reached = 1
;
; NOTES:
;   Compare minutes first, then seconds.
;-------------------------------------------------------------------------------
Time_Compare_MMSS:
    push acc
    push psw

    mov a, Control_FSM_state
    cjne a, #3, TC_Not_Soak

; ============================================================
; STATE 3: SOAK TIME COMPARISON
; ============================================================
    jbc state_change_signal_TC, TC_Soak_Start_Record
    sjmp TC_Soak_Comparing

TC_Soak_Start_Record:
    ; Calculate end time = current_time + soak_time
    mov a, current_time_minute
    add a, soak_time_minute
    mov soak_end_time_minute, a

    mov a, current_time_sec
    add a, soak_time_sec
    mov soak_end_time_sec, a

    ; Check for seconds overflow (>= 60)
    clr c
    subb a, #60
    jc TC_Soak_Comparing           ; No overflow, skip adjustment

    ; Overflow: adjust seconds and add 1 to minutes
    mov soak_end_time_sec, a
    inc soak_end_time_minute

TC_Soak_Comparing:
    ; Compare minutes first
    mov  a, current_time_minute
    clr  c
    subb a, soak_end_time_minute
    jc   TC_Done                   ; current_min < end_min -> not reached
    jnz  TC_Soak_Reached           ; current_min > end_min -> reached

    ; Minutes equal -> compare seconds
    mov  a, current_time_sec
    clr  c
    subb a, soak_end_time_sec
    jc   TC_Done                   ; current_sec < end_sec -> not reached
                                   ; current_sec >= end_sec -> fall through to reached

TC_Soak_Reached:
    setb soak_time_reached
    sjmp TC_Done

; ============================================================
; STATE 5: REFLOW TIME COMPARISON
; ============================================================
TC_Not_Soak:
    mov a, Control_FSM_state
    cjne a, #5, TC_Done

    jbc state_change_signal_TC, TC_Reflow_Start_Record
    sjmp TC_Reflow_Comparing

TC_Reflow_Start_Record:
    ; Calculate end time = current_time + reflow_time
    mov a, current_time_minute
    add a, reflow_time_minute
    mov reflow_end_time_minute, a

    mov a, current_time_sec
    add a, reflow_time_sec
    mov reflow_end_time_sec, a

    ; Check for seconds overflow (>= 60)
    clr c
    subb a, #60
    jc TC_Reflow_Comparing         ; No overflow, skip adjustment

    ; Overflow: adjust seconds and add 1 to minutes
    mov reflow_end_time_sec, a
    inc reflow_end_time_minute

TC_Reflow_Comparing:
    ; Compare minutes first
    mov  a, current_time_minute
    clr  c
    subb a, reflow_end_time_minute
    jc   TC_Done                   ; current_min < end_min -> not reached
    jnz  TC_Reflow_Reached         ; current_min > end_min -> reached

    ; Minutes equal -> compare seconds
    mov  a, current_time_sec
    clr  c
    subb a, reflow_end_time_sec
    jc   TC_Done                   ; current_sec < end_sec -> not reached
                                   ; current_sec >= end_sec -> fall through to reached

TC_Reflow_Reached:
    setb reflow_time_reached

TC_Done:
    pop  psw
    pop  acc
    ret

;-------------------------------------------------------------------------------;
; Temp_Compare
; Checks if we have reached the user's target temperatures.
; Only compares relevant temperature based on current Control_FSM_state:
;   State 2: Sets 'soak_temp_reached' if current_temp >= soak_temp
;   State 4: Sets 'reflow_temp_reached' if current_temp >= reflow_temp
;   State 6: Sets 'cooling_temp_reached' if current_temp < 100°C
;-------------------------------------------------------------------------------;
Temp_Compare:
    push acc
    push psw
    push AR0
    push AR1
    push AR2
    
    mov a, Control_FSM_state
    
    ; --- CHECK STATE 2: SOAK TEMP ---
    cjne a, #2, Temp_Compare_Check_State4
    sjmp Check_Soak_Threshold
    
Temp_Compare_Check_State4:
    ; --- CHECK STATE 4: REFLOW TEMP ---
    cjne a, #4, Temp_Compare_Check_State6
    sjmp Check_Reflow_Threshold
    
Temp_Compare_Check_State6:
    ; --- CHECK STATE 6: COOLING TEMP ---
    cjne a, #6, Temp_Compare_Done
    sjmp Check_Cooling_Threshold

Check_Soak_Threshold:
    ; Copy current_temp to X
    mov R0, #current_temp
    mov R1, #x
    lcall Copy4_Bytes_R0_to_R1

    ; Copy soak_temp to Y
    mov R0, #soak_temp
    mov R1, #y
    lcall Copy4_Bytes_R0_to_R1

    ; Compare: Is X (Current) < Y (Target)?
    lcall x_lt_y
    jb mf, Temp_Compare_Done          ; If Current < Target, not reached yet
    
    ; If Current >= Target
    setb soak_temp_reached
    sjmp Temp_Compare_Done

Check_Reflow_Threshold:
    ; Copy current_temp to X
    mov R0, #current_temp
    mov R1, #x
    lcall Copy4_Bytes_R0_to_R1

    ; Copy reflow_temp to Y
    mov R0, #reflow_temp
    mov R1, #y
    lcall Copy4_Bytes_R0_to_R1

    ; Compare
    lcall x_lt_y
    jb mf, Temp_Compare_Done          ; If Current < Target, not reached yet
    
    ; If Current >= Target
    setb reflow_temp_reached
    sjmp Temp_Compare_Done

Check_Cooling_Threshold:
    ; Copy current_temp to X
    mov R0, #current_temp
    mov R1, #x
    lcall Copy4_Bytes_R0_to_R1
    
    Load_y(100)                        ; Cooling target = 100°C
    lcall x_lt_y
    jnb mf, Temp_Compare_Done         ; If temp >= 100, not cooled yet
    
    ; If Current < 100°C
    setb cooling_temp_reached

; ---------------------------------------------------------
Temp_Compare_Done:
    pop AR2
    pop AR1
    pop AR0
    pop psw
    pop acc
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

;-------------------------------------------------------------------------------

;-------------------------------------------------------------------------------;
; Abort condition safety check Temperature time
;
; PURPOSE:
;   Automatic cycle termination on error:
;   Abort if oven fails to reach at least 50C in first 60s (1 minute).
;
; TRIP CONDITION:
;   if (current_time_minute >= 1) AND (current_temp < 50C)
;       -> set tc_missing_abort
;       -> set stop_signal
;
; ASSUMPTIONS:
;   - current_time_sec (byte): seconds 0-59
;   - current_time_minute (byte): minutes counter
;   - current_temp is in DEGREES C (integer, 32-bit, little-endian)
;-------------------------------------------------------------------------------;
Safety_Check_TC:
    push acc
    push psw
    push AR0
    push AR1

    ; ---------------------------------------------------------
    ; GATEKEEPER: IGNORE UNLESS IN STATE 2 (RAMP TO SOAK)
    ; ---------------------------------------------------------
    mov a, Control_FSM_state
    cjne a, #2, Safety_TC_Exit_Bridge ; If State != 2, skip everything
    sjmp Safety_Logic_Proceed         ; If State == 2, do the check

Safety_TC_Exit_Bridge:
    ljmp Safety_TC_Done               ; Jump to the end

Safety_Logic_Proceed:
    ; If already aborted or startup window closed, do nothing
    jb   tc_missing_abort, Safety_TC_Done
    jnb  tc_startup_window, Safety_TC_Done

    ; Check: current_time_minute >= 1 ? (has 1 minute passed?)
    mov  a, current_time_minute
    jz   Safety_TC_Done               ; minute == 0, still waiting

    ; We reached 1 minute: close the startup window so it won't re-check later
    clr  tc_startup_window

    ; Now check: current_temp < 50 ?
    mov  R0, #current_temp
    mov  R1, #x
    lcall Copy4_Bytes_R0_to_R1

    Load_Y(50)
    lcall x_lt_y
    jnb  mf, Safety_TC_Done           ; temp >= 50 ? pass

    ; FAIL: at 1 minute, still below 50C ? abort
    clr  PWM_OUT
    setb tc_missing_abort
    setb stop_signal
    lcall Beep_Ten
    
    ; Force FSM to State 0 (Welcome)
    mov Control_FSM_state, #0
    
    ; Force UI to State 0 (Home Screen)
    mov Current_State, #0
    
    ; Trigger Screen Refresh
    setb state_change_signal          ; Tell loop to redraw "Welcome"

Safety_TC_Done:
    pop  AR1
    pop  AR0
    pop  psw
    pop  acc
    ret

; ============================================================
; BUZZER STARTUP FUNCTIONS
; ============================================================
;============================================================
; Beep_Judge
; Purpose: Trigger appropriate beep pattern based on events
;   - Beep once when state changes
;   - Beep five times when entering state 6 (cooling/finished)
;   - Beep ten times if tc_missing_abort = 1 (error)
;
; Call this in main loop after Control_FSM
;============================================================
Beep_Judge:
    push acc
    push psw

    ; --- Priority 1: Error condition (highest priority) ---
    jnb tc_missing_abort, Beep_Judge_Check_State6
    ; Error detected - beep 10 times (only once per error)
    jb beep_error_done, Beep_Judge_Done   ; Already beeped for this error?
    setb beep_error_done                   ; Mark as handled
    lcall Beep_Ten
    sjmp Beep_Judge_Done

Beep_Judge_Check_State6:
    ; --- Priority 2: Entering State 6 (finished) ---
    jnb state_change_beep_signal, Beep_Judge_Done  ; No state change? Exit
    
    mov a, Control_FSM_state
    cjne a, #6, Beep_Judge_Normal_Change
    ; Entering state 6 - beep 5 times
    clr state_change_beep_signal               ; Consume the signal
    lcall Beep_Five
    sjmp Beep_Judge_Done

Beep_Judge_Normal_Change:
    ; --- Priority 3: Normal state change - beep once ---
    clr state_change_beep_signal               ; Consume the signal
    lcall Beep_Once

Beep_Judge_Done:
    pop psw
    pop acc
    ret

Beep_Once:
    mov beep_count, #1
    sjmp Beep_Start

Beep_Five:
    mov beep_count, #5
    sjmp Beep_Start

Beep_Ten:
    mov beep_count, #10
    sjmp Beep_Start      ; [FIX] Added explicit jump for safety

Beep_Start:
    clr TR0              ; [FIX] Stop timer briefly to reset cleanly
    mov beep_state, #1   ; Set State to ON
    mov beep_tmr, #0     ; Reset Timer High Byte
    mov beep_tmr+1, #0   ; Reset Timer Low Byte
    setb ET0             ; [FIX] Ensure Interrupt is enabled
    setb TR0             ; START the 2kHz tone
    ret
;-------------------------------------------------------------------------------
; Buzzer beep Task 
; Purpose: beeps, holds, stop
; Buzzer task:
; Beep once when state changes
; Beep five times if finished
; Beep ten times if meets error
;============================================================
Beep_Task:
    jnb one_ms_beep_flag, Beep_Done
    clr one_ms_beep_flag

    mov a, beep_state
    jz Beep_Done

; ---- increment 16-bit timer ----
    inc beep_tmr
    mov a, beep_tmr
    jnz Beep_Check
    inc beep_tmr+1

Beep_Check:
    ; FUZZY TIMER CHECK
    ; Check if High Byte is non-zero (Time >= 256ms)
    mov a, beep_tmr+1
    jz Beep_Done        ; If 0, keep beeping

    ; --- Time Limit Reached ---
    mov beep_tmr, #0    ; Reset timer
    mov beep_tmr+1, #0

    mov a, beep_state
    cjne a, #1, Beep_Off_State

    ; State was 1 (ON) -> Turn OFF
    clr TR0             ; Hardware Silence
    mov beep_state, #2  ; Set State to OFF (Pause)
    ret

Beep_Off_State:
; ---- OFF finished -> decrement count / next ON ----
    dec beep_count
    mov a, beep_count
    jz  Beep_Stop

    mov beep_state, #1
    setb TR0
    ret

Beep_Stop:
    clr TR0
    mov beep_state, #0
    ret

Beep_Done:
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
	setb state_change_signal_TC
	ret
Control_FSM_state0:
    cjne a, #0, Control_FSM_state1
    clr beep_error_done
    jnb PB0_flag, Control_FSM_state0_ret  ; Check flag
    clr PB0_flag                 
    sjmp Control_FSM_state1_a
Control_FSM_state0_ret:
    ret

Control_FSM_state1_a:
    mov Control_FSM_state, #1
    mov Current_State, #0
    setb state_change_signal
	setb state_change_signal_TC
    setb state_change_beep_signal
	ret
Control_FSM_state1:
    cjne a, #1, Control_FSM_state2
    jnb PB0_flag, Control_FSM_state1_ret  ; Check flag
    clr PB0_flag                    
    sjmp Control_FSM_state2_a
Control_FSM_state1_ret:
    ret

; --- STATE 2: RAMP TO SOAK ---
Control_FSM_state2_a:
	mov Control_FSM_state, #2
	setb state_change_signal
	setb state_change_signal_TC
	setb state_change_signal_Count
    setb state_change_beep_signal
    clr soak_temp_reached
	ret
Control_FSM_state2:
    cjne a, #2, Control_FSM_state3
    jnb PB2_flag, State2_Check
    clr PB2_flag
    ljmp Control_FSM_state6_a ; Pause

State2_Check:
    jnb soak_temp_reached, State2_Ret
    
    ; --- Move to State 3 ---
    clr soak_temp_reached
    clr soak_time_reached
	sjmp Control_FSM_state3_a
State2_Ret:
    ret

; --- STATE 3: SOAK PHASE ---
Control_FSM_state3_a:
	mov Control_FSM_state, #3
	setb state_change_signal
	setb state_change_signal_TC
    setb state_change_beep_signal
	ret
Control_FSM_state3:
    cjne a, #3, Control_FSM_state4
    jnb PB2_flag, State3_Check
    clr PB2_flag
    ljmp Control_FSM_state6_a
State3_Check:
    jnb soak_time_reached, State3_Ret
    clr soak_time_reached
    sjmp Control_FSM_state4_a
State3_Ret:
    ret

; --- STATE 4: RAMP TO PEAK ---
Control_FSM_state4_a:
	mov Control_FSM_state, #4
	setb state_change_signal
	setb state_change_signal_TC
    setb state_change_beep_signal
	ret
Control_FSM_state4:
    cjne a, #4, Control_FSM_state5
    jnb PB2_flag, State4_Check
    clr PB2_flag
    ljmp Control_FSM_state6_a
State4_Check:
    jnb reflow_temp_reached, State4_Ret
    clr reflow_temp_reached
    clr reflow_time_reached
	sjmp Control_FSM_state5_a
State4_Ret:
    ret

; --- STATE 5: REFLOW PHASE ---
Control_FSM_state5_a:
	mov Control_FSM_state, #5
	setb state_change_signal
	setb state_change_signal_TC
    setb state_change_beep_signal
	ret
Control_FSM_state5:
    cjne a, #5, Control_FSM_state6
    jnb PB2_flag, State5_Check
    clr PB2_flag
    sjmp Control_FSM_state6_a
State5_Check:
    jnb reflow_time_reached, State5_Ret
    clr reflow_time_reached
    sjmp Control_FSM_state6_a
State5_Ret:
    ret

; --- STATE 6: COOLING ---
Control_FSM_state6_a:
	mov Control_FSM_state, #6
	setb state_change_signal
	setb state_change_signal_TC
	setb state_change_signal_Count
    setb state_change_beep_signal
    clr cooling_temp_reached
	ret
Control_FSM_state6:
    cjne a, #6, Control_FSM_state7
    ; Wait for Cooling Temp Reached
    jnb cooling_temp_reached, State6_Ret
    clr cooling_temp_reached
    sjmp Control_FSM_state7_a
State6_Ret:
    ret

; --- STATE 7: DONE ---
Control_FSM_state7_a:
	mov Control_FSM_state, #7
	setb state_change_signal
	setb state_change_signal_TC
    setb state_change_beep_signal
	ret
Control_FSM_state7:
    cjne a, #7, Control_FSM_done
    ; Let's assume you meant the physical button P1.0 like State 0
    jbc PB0_flag, Control_FSM_state7_pressed
	sjmp Control_FSM_done
Control_FSM_state7_pressed:
    ljmp Control_FSM_state0_a

Control_FSM_done:
    ret

; ================================================================
; UI & HELPER SUBROUTINES
; ================================================================

; ----------------------------------------------------------------
; MODULE: BRIDGE (Text to Integer Conversion)
; ----------------------------------------------------------------

;--------------------------
;keep updating varaibles
Update_FSM_Variables:
	push ACC
	push AR6
	push AR7
	mov a, Control_FSM_state
	cjne a, #1, Update_FSM_Variables_done

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
    mov soak_time_minute, R7
    mov soak_time_sec, R6

    ; --- 4. REFLOW TIME ---
    mov R0, #Buf_Refl_Time
    lcall Parse_Time_String
    mov reflow_time_minute, R7
    mov reflow_time_sec, R6

Update_FSM_Variables_done:
	pop AR7
	pop AR6
	pop ACC
    ret
;--------------------------

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

; --- Parse "MMSS" to ceconds ---
Parse_Time_String:
    ; Minutes tens
    mov A, @R0
    clr C
    subb A, #0x30
    mov B, #10
    mul AB
    mov R5, A
    inc R0

    ; Minutes ones
    mov A, @R0
    clr C
    subb A, #0x30
    add A, R5
    mov R5, A
    inc R0

    ; Seconds tens
    mov A, @R0
    clr C
    subb A, #0x30
    mov B, #10
    mul AB
    mov R4, A
    inc R0

    ; Seconds ones
    mov A, @R0
    clr C
    subb A, #0x30
    add A, R4
    mov R4, A

    ; Return minutes/seconds
	mov a, R5
    mov R7, a     ; minutes
	mov a, R4
    mov R6, a     ; seconds
    ret

; ----------------------------------------------------------------
; MODULE: BUTTON HANDLER (Mode Selection)
; ----------------------------------------------------------------
; Blocking wrapper for LCD clear (keeps old behavior just for this)
Wait_25ms_BLOCKING:
    lcall Wait_25ms
    jnc Wait_25ms_BLOCKING ; Keep jumping back until Done (C=1)
    ret

; ----------------------------------------------------------------
; MODULE: BUTTON HANDLER (Non-Blocking Debounce)
; ----------------------------------------------------------------
; Variables needed:
;   BTN_DEB_state   - state machine state (0-3)
;   BTN_DEB_timer   - debounce timer (incremented by ISR every 1ms)
;   BTN_DEB_id      - which button was pressed (1-4)
; ----------------------------------------------------------------

Check_Buttons:
    push ACC
    push PSW
    
    ; Only process in Control_FSM_state 1
    mov a, Control_FSM_state
    cjne a, #1, Check_Buttons_Done_bridge
    
    ; --- FORCE INPUT MODE ---
    orl P0, #055H   ; Sets P0.0, P0.2, P0.4, P0.6 to '1' (Input Mode)
    
    mov a, BTN_DEB_state
    sjmp BTN_DEB_state0

Check_Buttons_Done_bridge:
    ljmp Check_Buttons_Done

; ============================================================
; State 0: Wait for any button press
; ============================================================
BTN_DEB_state0:
    cjne a, #0, BTN_DEB_state1
    
    ; Check each button, record which one was pressed
    jnb BTN_SOAK_TEMP, BTN_Detect_SoakTemp
    jnb BTN_SOAK_TIME, BTN_Detect_SoakTime
    jnb BTN_REFL_TEMP, BTN_Detect_ReflTemp
    jnb BTN_REFL_TIME, BTN_Detect_ReflTime
    ljmp Check_Buttons_Done     ; No button pressed

BTN_Detect_SoakTemp:
    mov BTN_DEB_id, #1
    sjmp BTN_Start_Debounce
BTN_Detect_SoakTime:
    mov BTN_DEB_id, #2
    sjmp BTN_Start_Debounce
BTN_Detect_ReflTemp:
    mov BTN_DEB_id, #3
    sjmp BTN_Start_Debounce
BTN_Detect_ReflTime:
    mov BTN_DEB_id, #4
    sjmp BTN_Start_Debounce

BTN_Start_Debounce:
    mov BTN_DEB_timer, #0
    inc BTN_DEB_state
    sjmp Check_Buttons_Done

; ============================================================
; State 1: Debounce delay (wait 50ms)
; ============================================================
BTN_DEB_state1:
    cjne a, #1, BTN_DEB_state2
    mov a, BTN_DEB_timer
    cjne a, #50, Check_Buttons_Done   ; Wait 50ms
    inc BTN_DEB_state
    sjmp Check_Buttons_Done

; ============================================================
; State 2: Verify button still pressed
; ============================================================
BTN_DEB_state2:
    cjne a, #2, BTN_DEB_state3
    
    ; Check if the same button is still pressed
    mov a, BTN_DEB_id
    cjne a, #1, BTN_Verify_Check2
    jnb BTN_SOAK_TEMP, BTN_Verify_OK
    sjmp BTN_Verify_Fail
BTN_Verify_Check2:
    cjne a, #2, BTN_Verify_Check3
    jnb BTN_SOAK_TIME, BTN_Verify_OK
    sjmp BTN_Verify_Fail
BTN_Verify_Check3:
    cjne a, #3, BTN_Verify_Check4
    jnb BTN_REFL_TEMP, BTN_Verify_OK
    sjmp BTN_Verify_Fail
BTN_Verify_Check4:
    jnb BTN_REFL_TIME, BTN_Verify_OK
    ; Fall through to fail

BTN_Verify_Fail:
    mov BTN_DEB_state, #0           ; Was noise, reset
    sjmp Check_Buttons_Done

BTN_Verify_OK:
    inc BTN_DEB_state               ; Confirmed, wait for release
    sjmp Check_Buttons_Done

; ============================================================
; State 3: Wait for button release, then trigger action
; ============================================================
BTN_DEB_state3:
    cjne a, #3, Check_Buttons_Done
    
    ; Check if the button is released
    mov a, BTN_DEB_id
    cjne a, #1, BTN_Release_Check2
    jnb BTN_SOAK_TEMP, Check_Buttons_Done   ; Still pressed, wait
    sjmp BTN_Do_Action
BTN_Release_Check2:
    cjne a, #2, BTN_Release_Check3
    jnb BTN_SOAK_TIME, Check_Buttons_Done
    sjmp BTN_Do_Action
BTN_Release_Check3:
    cjne a, #3, BTN_Release_Check4
    jnb BTN_REFL_TEMP, Check_Buttons_Done
    sjmp BTN_Do_Action
BTN_Release_Check4:
    jnb BTN_REFL_TIME, Check_Buttons_Done
    ; Fall through to action

; ============================================================
; Button Released - Execute Action
; ============================================================
BTN_Do_Action:
    mov a, BTN_DEB_id
    
    cjne a, #1, BTN_Action_2
    mov Current_State, #1           ; Soak Temp
    sjmp BTN_Action_Complete
BTN_Action_2:
    cjne a, #2, BTN_Action_3
    mov Current_State, #2           ; Soak Time
    sjmp BTN_Action_Complete
BTN_Action_3:
    cjne a, #3, BTN_Action_4
    mov Current_State, #3           ; Refl Temp
    sjmp BTN_Action_Complete
BTN_Action_4:
    mov Current_State, #4           ; Refl Time

BTN_Action_Complete:
    mov Cursor_Idx, #0
    setb fullscreen_update_signal   ; Trigger screen redraw
    mov BTN_DEB_state, #0           ; Reset state machine

Check_Buttons_Done:
    pop PSW
    pop ACC
    ret
; ----------------------------------------------------------------
; MODULE: KEYPAD HANDLER (Input Logic)
; ----------------------------------------------------------------
Check_Keypad:
    mov a, Control_FSM_state
    cjne a, #1, Keypad_Exit

    ; If State is 0 (Home), ignore keypad
    mov A, Current_State
    jz Keypad_Exit
    
    lcall Keypad_Scan
    jnc Keypad_Exit         ; Carry = 0 means no key pressed

    mov A, R7
    cjne A, #14, Check_Hash ; 14 is Star (*)
    
    lcall Reset_Current_Buffer
    setb fullscreen_update_signal
    mov Cursor_Idx, #0
    ret

Check_Hash:
    mov A, R7
    cjne A, #12, Check_Numeric 
    ret                

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

    ;save to Buffer
    lcall Get_Current_Buffer_Addr
    mov A, Cursor_Idx
    add A, R0
    mov R0, A
    mov A, R5
    mov @R0, A
    inc Cursor_Idx

    ;Check cursor limits ---
    mov A, Current_State
    cjne A, #1, Check_Limit_Time_1
    sjmp Limit_Temp_3

Check_Limit_Time_1:
    cjne A, #3, Limit_Time_4
    sjmp Limit_Temp_3

Limit_Temp_3:
    mov A, Cursor_Idx
    cjne A, #3, Do_Refresh
    dec Cursor_Idx          
    sjmp Do_Refresh

Limit_Time_4:
    mov A, Cursor_Idx
    cjne A, #4, Do_Refresh
    dec Cursor_Idx         
    sjmp Do_Refresh

Do_Refresh:
    setb fullscreen_update_signal
    ret

Symbol_Key_Ignored:
    ret
Keypad_Exit:
    ret

;-------------------------------------------------------------------------------
; hardware
;-------------------------------------------------------------------------------
Keypad_Scan:
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
    lcall Wait_25ms_BLOCKING
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

; Key Mapping
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
    ; 1. Check if we are already waiting
    jb wait25_active, Check_Timer_Status
    
    ; 2. Check if we just finished
    jnb wait25_done, Start_New_Timer
    
    ; 3. Timer is done
    clr wait25_done
    setb C          ; Carry = 1 means done
    ret

Start_New_Timer:
    ; 4. Start a new 25ms wait
    mov wait25_count, #0
    setb wait25_active
    clr C           ; Carry = 0 means not yet
    ret

Check_Timer_Status:
    ; 5. Still waiting... return False immediately
    clr C           ; Carry = 0 means "Not Done Yet"
    ret

;-------------------------------------------------------------------------------
; reset logic
;-------------------------------------------------------------------------------
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
    
; ================================================================
; MODULE: THERMOCOUPLE ADC DRIVER (WITH NOISE SUPPRESSION & JUMP FIX)
; ================================================================
Read_Thermocouple:
    lcall Wait_25ms
    jc Proceed_Reading
    ret 

Proceed_Reading:
    mov A, TCON      
    anl A, #0x10    
    push acc         
    clr TR0         

    mov ADC_C, #0x80    
    nop
    nop
    mov ADC_C, #0x01   
    
    mov R5, #250
ADC_Settle_Loop:
    nop
    nop
    djnz R5, ADC_Settle_Loop
    
    mov x+0, ADC_L
    mov x+1, ADC_H
    mov x+2, #0
    mov x+3, #0
    
    mov a, x+1
    anl a, #0x0F
    mov x+1, a
    
    pop acc         
    jz Skip_Restore 
    setb TR0       
Skip_Restore:

	; as per our volatge reference (measured using the DMM)
    Load_y(4118)
    lcall mul32       

    mov ADC_C, #0x04   
    mov y+0, ADC_L      
    mov y+1, ADC_H      
    mov y+2, #0
    mov y+3, #0
    mov ADC_C, #0x00   
    
    lcall div32         
    Load_Y(100)
    lcall mul32
    Load_y(1323) ;using our amplifiers resistance ratio and 41uV   
    lcall div32    
    Load_y(COLD_JUNCTION_TEMP)
    lcall add32     
    
    mov current_temp+0, x+0
    mov current_temp+1, x+1
    mov current_temp+2, x+2
    mov current_temp+3, x+3

    ret
    
;-------------------------------------------------------------------------------
; power control
;-------------------------------------------------------------------------------
Power_Control:
    mov power_output+0, #0
    mov power_output+1, #0
    mov power_output+2, #0
    mov power_output+3, #0

    mov a, Control_FSM_state

    ; --- State 2: RAMP TO SOAK ---
    cjne a, #2, PC_Check_Soak
    sjmp Set_Max_Power

PC_Check_Soak:
    ; --- State 3: SOAK PHASE ---
    cjne a, #3, PC_Check_Ramp_Reflow
    jb soak_temp_reached, PC_Done 
    sjmp Set_20_Percent_Power     

PC_Check_Ramp_Reflow:
    ; --- State 4: RAMP TO REFLOW ---
    cjne a, #4, PC_Check_Reflow
    sjmp Set_Max_Power

PC_Check_Reflow:
    ; --- State 5: REFLOW PHASE ---
    cjne a, #5, PC_Done
    jb reflow_temp_reached, PC_Done
    sjmp Set_20_Percent_Power

PC_Done:
    ret

Set_Max_Power:
    ; Load 1500 (0x05DC) = 100% Duty Cycle
    mov power_output+0, #0xDC
    mov power_output+1, #0x05
    mov power_output+2, #0
    mov power_output+3, #0
    ret

Set_20_Percent_Power:
    ; Load 300 (0x012C) = 20% Duty Cycle
    mov power_output+0, #0x2C
    mov power_output+1, #0x01
    mov power_output+2, #0
    mov power_output+3, #0
    ret
;--------------------------------------------------------------
; set servo angle according to the state
; call servo control function every 1ms
;--------------------------------------------------------------
call_servo_control:
	; check current state and change servo angle
	mov a, Control_FSM_state
	
	; handle state 0
	cjne a, #0, servo_state1
	clr servo_angle_zero ; close door at state 0
	sjmp check_servo_flag

	; handle state 1
	servo_state1:
	cjne a, #1, servo_state2
	setb servo_angle_zero ; open door at state 1
	sjmp check_servo_flag

	; handle state 2
	servo_state2:
	cjne a, #2, servo_state3
	clr servo_angle_zero ; close door at state 2
	sjmp check_servo_flag

	; handle state 3
	servo_state3:
	cjne a, #3, servo_state4
	clr servo_angle_zero ; close door at state 3
	sjmp check_servo_flag

	; handle state 4
	servo_state4:
	cjne a, #4, servo_state5
	clr servo_angle_zero ; close door at state 4
	sjmp check_servo_flag

	; handle state 5
	servo_state5:
	cjne a, #5, servo_state6
	clr servo_angle_zero ; close door at state 5
	sjmp check_servo_flag

	; handle state 6
	servo_state6:
	cjne a, #6, servo_state7
	clr servo_angle_zero ; close door at state 6
	sjmp check_servo_flag

	; handle state 7
	servo_state7:
	setb servo_angle_zero ; open door at state 7

check_servo_flag:
	; check 1 ms flag
	jbc one_millisecond_flag_servo, run_servo_control
	ret

run_servo_control:
	lcall servo_control
	ret

;---------------------------------------------------------------
; servo control
; generate a 20 ms period pwm signal to control the servo motor
; able to make the servo motor stay at 0 degree and 180 degree
;---------------------------------------------------------------
servo_control:
    setb LEDRA.5
	push acc
	push psw
	mov a, servo_pwm_counter ; move servo counter to accumulator
	inc A ; a += 1
	cjne a, #SERVO_PERIOD, servo_pwm_angle_compare ; jump if wrapup not needed
	mov a, #0

servo_pwm_angle_compare: ; read target angle
	mov servo_pwm_counter, A
	jb servo_angle_zero, set_zero_degree ; set servo motor to 0 degree
	; set servo motor to 180 degrees
	mov a, servo_pwm_counter
	clr c
	subb a, #SERVO_180
	jc servo_pwm_set_high ; set high if servo pwm counter smaller than 180 degrees duty cycle
	sjmp servo_pwm_set_low ; set low if greater

set_zero_degree:
	; set servo motor to 0 degree
	mov a, servo_pwm_counter
	clr c
	subb a, #SERVO_0
	jc servo_pwm_set_high ; set high if servo pwm counter smaller than 0 degrees duty cycle
	sjmp servo_pwm_set_low ; set low if greater

servo_pwm_set_high:
	; set pwm pin high
	setb SERVO_OUT
	sjmp servo_control_done

servo_pwm_set_low:
	; set pwm pin low
	clr SERVO_OUT

servo_control_done:
	pop psw
	pop acc
	ret

;-------------------------------------------------------------------------------
; power_control
;-------------------------------------------------------------------------------
; Determine the power output based on current state and current temperature 
; input parameter: Control_FSM_state
;-------------------------------------------------------------------------------
; Update LED indicators based on power_output
; 0%  -> all off
; <50% -> LED_LEFT on
; 50%-99% -> LED_LEFT + LED_MID on
; >=100% -> LED_LEFT + LED_MID + LED_RIGHT on
Update_Power_LEDs:
    ; Check for exact 0 (all bytes zero)
    mov a, power_output
    orl a, power_output+1
    orl a, power_output+2
    orl a, power_output+3
    jz power_leds_low

    mov x, power_output
    mov x+1, power_output+1
    mov x+2, power_output+2
    mov x+3, power_output+3

    Load_Y(HALF_POWER)
    lcall x_lt_y
    jb mf, power_leds_low

    Load_Y(MAX_POWER)
    lcall x_lt_y
    jb mf, power_leds_mid

    sjmp power_leds_high

power_leds_low:
    clr  LED_LEFT
    clr  LED_MID
    clr  LED_RIGHT
    ret

power_leds_mid:
    setb LED_LEFT
    setb LED_MID
    clr  LED_RIGHT
    ret

power_leds_high:
    setb LED_LEFT
    setb LED_MID
    setb LED_RIGHT
    ret

proportional_power_control:
	mov a, Control_FSM_state

state0_power_control:
	; idle
	; 0% power
	cjne a, #0, state1_power_control
	mov power_output, #low(NO_POWER)
	mov power_output+1, #low(NO_POWER)
	mov power_output+2, #0
	mov power_output+3, #0
	ljmp power_control_done

state1_power_control:
	; idle
	; 0% power
	cjne a, #1, state2_power_control
	mov power_output, #low(NO_POWER)
	mov power_output+1, #low(NO_POWER)
	mov power_output+2, #0
	mov power_output+3, #0
	ljmp power_control_done
	
state2_power_control:
	; ramp to soak, ramp to ~150C
	; 100% power
	cjne a, #2, state3_power_control
    
    mov x, soak_temp
    mov x+1, soak_temp+1
    mov x+2, soak_temp+2
    mov x+3, soak_temp+3
    Load_Y(5)
    lcall sub32 
    ; now x holds soak_temp-5
    ; turn power to 20% when current_temp > soak_temp-5

    mov y, current_temp
    mov y+1, current_temp+1
    mov y+2, current_temp+2
    mov y+3, current_temp+3

    clr mf
    lcall x_gteq_y
    
    jbc mf, state_2_full_power ; turn on full power when current_temp <= soak_temp-5
    ; turn on 20% power when current_temp > soak_temp-5
    mov power_output, #low(BASE_POWER)
    mov power_output+1, #high(BASE_POWER)
    mov power_output+2, #0
    mov power_output+3, #0
    ljmp power_control_done

state_2_full_power:
	mov power_output, #low(MAX_POWER)
	mov power_output+1, #high(MAX_POWER)
	mov power_output+2, #0
	mov power_output+3, #0
	ljmp power_control_done

state3_power_control:
	; soak period, hold at 150C
	; 20% base power + proportional calculated power
	cjne a, #3, jump_state4_power_control
	sjmp state3_power_control_calculation

jump_state4_power_control:
	ljmp state4_power_control

state3_power_control_calculation:
	; move soak_temp to x
	mov x, soak_temp
	mov x+1, soak_temp+1
	mov x+2, soak_temp+2
	mov x+3, soak_temp+3
	; move current_temp to y
	mov y, current_temp
	mov y+1, current_temp+1
	mov y+2, current_temp+2
	mov y+3, current_temp+3

	; compare between soak_temp and current_temp
	clr mf
	lcall x_gteq_y
	jbc mf, st_sub_ct
	; current_temp - soak_temp if st < ct
	clr soak_temp_greater
	; move current_temp to y
	mov y, soak_temp
	mov y+1, soak_temp+1
	mov y+2, soak_temp+2
	mov y+3, soak_temp+3
	; move current_temp to x
	mov x, current_temp
	mov x+1, current_temp+1
	mov x+2, current_temp+2
	mov x+3, current_temp+3
	lcall sub32
	mov soak_temp_diff, x
	mov soak_temp_diff+1, x+1
	mov soak_temp_diff+2, x+2
	mov soak_temp_diff+3, x+3
	sjmp proportional_input_soak

st_sub_ct:
	; soak_temp - current_temp
	setb soak_temp_greater
	lcall sub32
	mov soak_temp_diff, x
	mov soak_temp_diff+1, x+1
	mov soak_temp_diff+2, x+2
	mov soak_temp_diff+3, x+3

proportional_input_soak:
	; proportaional block calculation	
	; move soak_temp_diff to x
	mov x, soak_temp_diff
	mov x+1, soak_temp_diff+1
	mov x+2, soak_temp_diff+2
	mov x+3, soak_temp_diff+3
	; move proportional gain to y
	Load_Y(KP)
	lcall mul32 ; proportional_output = proportional_gain * difference
	
	mov proportional_gain_var, x
	mov proportional_gain_var+1, x+1
	mov proportional_gain_var+2, x+2
	mov proportional_gain_var+3, x+3

	; base_power + soak_power when soak_temp > current_temp
	jnb soak_temp_greater, sub_proportional_soak
	mov x, proportional_gain_var
	mov x+1, proportional_gain_var+1
	mov x+2, proportional_gain_var+2
	mov x+3, proportional_gain_var+3
	Load_Y(BASE_POWER)
	lcall add32
	; x now holds the power output before the saturator
	mov proportional_gain_var, x
	mov proportional_gain_var+1, x+1
	mov proportional_gain_var+2, x+2
	mov proportional_gain_var+3, x+3
	sjmp saturator_soak

sub_proportional_soak:
	; base_power - soak_power when soak_temp <= current_temp
	Load_X(BASE_POWER)
	mov y, proportional_gain_var
	mov y+1, proportional_gain_var+1
	mov y+2, proportional_gain_var+2
	mov y+3, proportional_gain_var+3

	; compare whether base_power < proportional_gain_var
	clr mf
	lcall x_lt_y ; set mf to 1 if base_power < proportional_gain_var, clamp output to 0
	jnb mf, bp_gteq_pgv
	mov proportional_gain_var, #low(NO_POWER)
	mov proportional_gain_var+1, #high(NO_POWER)
	mov proportional_gain_var+2, #0
	mov proportional_gain_var+3, #0
	sjmp saturator_soak

bp_gteq_pgv:
	; calculate subtracted gain
	lcall sub32
	; x now holds the power output before the saturator
	mov proportional_gain_var, x
	mov proportional_gain_var+1, x+1
	mov proportional_gain_var+2, x+2
	mov proportional_gain_var+3, x+3

saturator_soak:
	; proportional_gain_var now holds the power output before the saturator
	; saturate power output to max power
	mov x, proportional_gain_var
	mov x+1, proportional_gain_var+1
	mov x+2, proportional_gain_var+2
	mov x+3, proportional_gain_var+3

	Load_Y(MAX_POWER)

	clr mf
	lcall x_gt_y ; set mf to 1 if calculated power output greater than max power
	jb mf, saturated_soak
	; set power_output to calculated power if not saturated
	mov power_output, proportional_gain_var
	mov power_output+1, proportional_gain_var+1
	mov power_output+2, proportional_gain_var+2
	mov power_output+3, proportional_gain_var+3
	ljmp power_control_done

saturated_soak:
	mov power_output, #low(MAX_POWER)
	mov power_output+1, #high(MAX_POWER)
	mov power_output+2, #0
	mov power_output+3, #0
	ljmp power_control_done


state4_power_control:
	; ramp to reflow, max power
	cjne a, #4, state5_power_control
	mov power_output, #low(MAX_POWER)
	mov power_output+1, #high(MAX_POWER)
	mov power_output+2, #0
	mov power_output+3, #0
	ljmp power_control_done

state5_power_control:
	; reflow 20% base power
	cjne a, #5, state6_power_control
	mov power_output, #low(BASE_POWER)  
	mov power_output+1, #high(BASE_POWER)
	mov power_output+2, #0
	mov power_output+3, #0
	ljmp power_control_done

state6_power_control:
	; cooling 0% power
	cjne a, #6, state_7_power_control
	mov power_output, #low(NO_POWER)
	mov power_output+1, #high(NO_POWER)
	mov power_output+2, #0
	mov power_output+3, #0
	ljmp power_control_done

state_7_power_control:
	; idle 0% power
	mov power_output, #low(NO_POWER)
	mov power_output+1, #high(NO_POWER)
	mov power_output+2, #0
	mov power_output+3, #0

power_control_done:
	ret

;----------------------------------------------------------------------------------------
; function for playing the boot up music
music:
	; 1
	lcall playC
	lcall delayHalfSec
	; 1
	lcall playC
	lcall delayHalfSec
	; 5
	lcall playG
	lcall delayHalfSec
	; 5
	lcall playG
	lcall delayHalfSec
	; 6
	lcall PlayA
	lcall delayHalfSec
	; 6
	lcall PlayA
	lcall delayHalfSec
	; 5
	lcall playG
	lcall delayHalfSec
	; 4
	lcall playF
	lcall delayHalfSec

	; 4
	lcall playF
	lcall delayHalfSec
	; 3
	lcall playE
	lcall delayHalfSec

	; 3
	lcall playE
	lcall delayHalfSec

	; 2
	lcall playD
	lcall delayHalfSec

	; 2
	lcall playD
	lcall delayHalfSec
	; 1
	lcall playC
	lcall delayHalfSec
	
	lcall delayHalfSec
	ret

;------------------------------------
; Play_C_0p5s
; Plays ~523 Hz on P1.7 for 0.5 seconds
;------------------------------------
playC:
    MOV R7, #20        ; outer loop counter

OUTER_LOOP:
    MOV R6, #26        ; inner loop counter

INNER_LOOP:
    CPL SOUND_OUT           ; toggle buzzer pin
    LCALL Delay_C      ; ~960 ?s delay
    DJNZ R6, INNER_LOOP

    DJNZ R7, OUTER_LOOP

    CLR SOUND_OUT           ; stop sound (pin low)
    RET
    
Delay_C:
    MOV R5, #24        ; 24 ? 40 ?s = 960 ?s
DELAY_LOOP:
    LCALL Wait40uSec
    DJNZ R5, DELAY_LOOP
    RET
    

;------------------------------------
; Play_G_0p5s
; Plays ~784 Hz on P1.7 for 0.5 seconds
;------------------------------------
playG:
    MOV R7, #31        ; outer loop

OUTER_G:
    MOV R6, #25        ; inner loop 31 ? 25 = 775 toggles

INNER_G:
    CPL SOUND_OUT           ; toggle buzzer pin
    LCALL Delay_G      ; ~640 ?s delay
    DJNZ R6, INNER_G

    DJNZ R7, OUTER_G

    CLR SOUND_OUT           ; stop sound
    RET
    
Delay_G:
    MOV R5, #16        ; 16 ? 40 ?s = 640 ?s
DELAY_G_LOOP:
    LCALL Wait40uSec
    DJNZ R5, DELAY_G_LOOP
    RET
    	
 
 ;------------------------------------
; Play_A_0p5s
; Plays ~880 Hz on P1.7 for 0.5 seconds
;------------------------------------
playA:
    MOV R7, #34        ; outer loop counter

OUTER_A:
    MOV R6, #26        ; inner loop ? 34 ? 26 = 884 toggles

INNER_A:
    CPL SOUND_OUT           ; toggle buzzer pin
    LCALL Delay_A      ; ~560 ?s delay
    DJNZ R6, INNER_A

    DJNZ R7, OUTER_A

    CLR SOUND_OUT          ; stop sound
    RET

Delay_A:
    MOV R5, #14        ; 14 ? 40 ?s = 560 ?s
DELAY_A_LOOP:
    LCALL Wait40uSec
    DJNZ R5, DELAY_A_LOOP
    RET

;------------------------------------
; Play_F_0p5s
; Plays ~698 Hz on P1.7 for 0.5 seconds
;------------------------------------
playF:
    MOV R7, #26        ; outer loop

OUTER_F:
    MOV R6, #27        ; inner loop ? 26 ? 27 = 702 toggles

INNER_F:
    CPL SOUND_OUT          ; toggle buzzer pin
    LCALL Delay_F      ; ~720 ?s delay
    DJNZ R6, INNER_F

    DJNZ R7, OUTER_F

    CLR SOUND_OUT           ; stop sound
    RET

Delay_F:
    MOV R5, #18        ; 18 ? 40 ?s = 720 ?s
DELAY_F_LOOP:
    LCALL Wait40uSec
    DJNZ R5, DELAY_F_LOOP
    RET


;------------------------------------
; Play_E_0p5s
; Plays ~659 Hz on P1.7 for 0.5 seconds
;------------------------------------
playE:
    MOV R7, #26        ; outer loop

OUTER_E:
    MOV R6, #25        ; inner loop ? 26 ? 25 = 650 toggles

INNER_E:
    CPL SOUND_OUT           ; toggle buzzer pin
    LCALL Delay_E      ; ~760 ?s delay
    DJNZ R6, INNER_E

    DJNZ R7, OUTER_E

    CLR SOUND_OUT          ; stop sound
    RET
    
Delay_E:
    MOV R5, #19        ; 19 ? 40 ?s = 760 ?s
DELAY_E_LOOP:
    LCALL Wait40uSec
    DJNZ R5, DELAY_E_LOOP
    RET


;------------------------------------
; Play_D_0p5s
; Plays ~587 Hz on P1.7 for 0.5 seconds
;------------------------------------
playD:
    MOV R7, #25        ; outer loop

OUTER_D:
    MOV R6, #24        ; inner loop ? 25 ? 24 = 600 toggles

INNER_D:
    CPL SOUND_OUT          ; toggle buzzer pin
    LCALL Delay_D      ; ~840 ?s delay
    DJNZ R6, INNER_D

    DJNZ R7, OUTER_D

    CLR SOUND_OUT         ; stop sound
    RET
    
Delay_D:
    MOV R5, #21        ; 21 ? 40 ?s = 840 ?s
DELAY_D_LOOP:
    LCALL Wait40uSec
    DJNZ R5, DELAY_D_LOOP
    RET

delayHalfSec:
	mov	R2, #125
	lcall WaitmilliSec
	;lcall WaitmilliSec
	ret

;---------------------------------;
; Wait 'R2' milliseconds, blocking;
;---------------------------------;
WaitmilliSec:
    push AR0
    push AR1
loop3: mov R1, #40
loop2: mov R0, #104
loop1: djnz R0, loop1 ; 4 cycles->4*60.24ns*104=25.0us
    djnz R1, loop2 ; 25us*40=1.0ms
    djnz R2, loop3 ; number of millisecons to wait passed in R2
    pop AR1
    pop AR0
    ret
;-------------------------------------------------------------------------------

;------------------------------------------------------------------------------
; dc_control
; turns on the dc motor in cooling stage
;------------------------------------------------------------------------------
dc_control:
    mov a, Control_FSM_state
	
	; handle state 0
	cjne a, #0, dc_state1
	clr DC_OUT
    sjmp dc_control_done

	; handle state 1
	dc_state1:
	cjne a, #1, dc_state2
	clr DC_OUT
    sjmp dc_control_done

	; handle state 2
	dc_state2:
	cjne a, #2, dc_state3
	clr DC_OUT
    sjmp dc_control_done

	; handle state 3
	dc_state3:
	cjne a, #3, dc_state4
	clr DC_OUT
    sjmp dc_control_done

	; handle state 4
	dc_state4:
	cjne a, #4, dc_state5
	clr DC_OUT
    sjmp dc_control_done

	; handle state 5
	dc_state5:
	cjne a, #5, dc_state6
	clr DC_OUT
    sjmp dc_control_done

	; handle state 6
	dc_state6:
    cjne a, #6, dc_state7
	setb DC_OUT
    sjmp dc_control_done

	; handle state 7
	dc_state7:
	clr DC_OUT  

dc_control_done:
    ret

;-------------------------------------------------------------------------------;
;         Main program.          
;-------------------------------------------------------------------------------;
main:

    ; --------------------------------------------------------
    ; 1. SAFETY SHUTDOWN
    ; --------------------------------------------------------
    clr EA              ; FORCE Interrupts OFF immediately
    mov SP, #0xC0       ; Reset Stack Pointer to safe location
    
    ; --------------------------------------------------------
    ; THE "DIRTY DELAY" (Fixes Reset Garbage)
    ; We burn ~100ms here using a raw loop. 
    ; We cannot use timers yet because they aren't initialized.
    ; --------------------------------------------------------
    mov R0, #250
Reset_Delay_Outer:
    mov R1, #255
Reset_Delay_Inner:
    djnz R1, Reset_Delay_Inner
    djnz R0, Reset_Delay_Outer
    
    ; --- PORT CONFIGURATION ---
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
    mov P4MOD, #00000001B

    ; Turn off all the LEDs
    mov LEDRA, #0 ; LEDRA is bit addressable
    mov LEDRB, #0 ; LEDRB is NOT bit addresable

    ; Enable Global interrupts
    setb EA  

	; FSM initial states
	mov SEC_FSM_state, #0
	mov Control_FSM_state, #0
	mov Current_State, #0
	; FSM timers initialization
	mov SEC_FSM_timer, #0
	; time counters initialization
	mov current_time_sec, #0
	mov current_time_minute, #0
	mov soak_time_sec, #0
	mov soak_time_minute, #0
	mov reflow_time_sec, #0
	mov reflow_time_minute, #0
	mov soak_end_time_sec, #0
	mov soak_end_time_minute, #0
	mov reflow_end_time_sec, #0
	mov reflow_end_time_minute, #0
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
	; FSM Buttons push button init
	mov	PB0_DEB_state, #0
	mov	PB2_DEB_state, #0
	mov	PB0_DEB_timer, #0
	mov	PB2_DEB_timer, #0
    ; [FIX] ADD THIS BLOCK TO STOP STARTUP BEEP
    mov beep_state, #0
    mov beep_count, #0
    mov beep_tmr, #0
    mov beep_tmr+1, #0
    ; Buttons 
    mov BTN_DEB_state, #0
    mov BTN_DEB_timer, #0
    mov BTN_DEB_id, #0
    mov servo_pwm_counter, #0
    mov rx_idx, #0
    mov rx_ready, #0
    clr one_ms_beep_flag
    clr TR0 ; Force buzzer hardware OFF

	; Clear all the flags
	clr SOUND_OUT
    clr beep_error_done
	clr tc_missing_abort
	clr stop_signal
	clr PB0_flag
	clr PB1_flag
	clr PB2_flag
	clr one_second_flag
	clr one_second_lcd_flag
	clr config_finish_signal
    clr time_count_doing_signal
    clr fullscreen_update_signal
	clr soak_temp_reached
	clr soak_time_reached
	clr reflow_temp_reached
	clr reflow_time_reached
	clr cooling_temp_reached
    clr state_change_signal_TC
	clr state_change_signal_Count
    clr state_change_beep_signal
    clr one_millisecond_flag_servo
    clr remote_config_mode
    clr DC_OUT
    ; Set bit
	setb state_change_signal
    setb tc_startup_window

    lcall Timer0_Init
    lcall Timer2_Init
    lcall ELCD_4BIT
    ;----- Two new lines I added to initialize the UI
    lcall Init_All_Buffers
    lcall Initialize_Serial_Port
    lcall music
;-------------------------------------------------------------------------------;
; while(1) loop
;-------------------------------------------------------------------------------;
loop:

	lcall SEC_FSM

	; Check the FSM for the overall control flow of the reflow process
    lcall Control_FSM

    ; Check the FSM for PB01 debounce
    lcall PB0_DEB
	lcall PB2_DEB
    
    ; Added to take temp readings
    lcall Read_Thermocouple
    
    ; 1. Check if we reached temp (Observer)
    lcall Temp_Compare
    
    ; 2. Decide heater power based on flags (Driver)
    ;lcall Power_Control
    lcall proportional_power_control
    
    lcall Safety_Check_TC

	lcall Time_Counter

	; Update Variables (times and temp)
	lcall Update_FSM_Variables

    ; GUI Interface polling uart port
    lcall Serial_RX_Pump
    lcall Serial_Process_Line

	; Update while at state 1
	; LCD
	lcall Update_Screen_Full 
	; Buttons
	lcall Check_Buttons 
	; PB0pad
    lcall Check_Keypad

    ; Update the LCD display based on the current state
    lcall LCD_Display_Update_func

	lcall Time_Compare_MMSS

    ; Update the pwm output for the ssr
    lcall PWM_Wave 
	; Update the Buzzer 
	lcall Beep_Task
    ; Update the pwm output for the servo
    lcall call_servo_control

    lcall Beep_Judge

    lcall dc_control

    ; After initialization the program stays in this 'forever' loop
    ljmp loop
;-------------------------------------------------------------------------------;
END