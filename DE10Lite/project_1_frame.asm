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
$include(MODMAX10)
;ADC_C DATA 0xa1
;ADC_L DATA 0xa2
;ADC_H DATA 0xa3
$include(math32.asm) ; 
$LIST
; ----------------------------------------------------------------------------------------------;
; Data Segment 0x30 -- 0x7F  (overall 79d bytes available)
dseg at 0x30
current_time_sec:     ds 1
current_time_minute:  ds 1
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
wait25_count: ds 1  

current_time: ds 4 ;
soak_time:    ds 4 ;
reflow_time:  ds 4 ;

power_output:  ds 4 ;
pwm_counter: ds 4 ; counter for pwm (0-1500)

KEY1_DEB_timer: ds 1
SEC_FSM_timer:  ds 1
KEY1_DEB_state:    ds 1
SEC_FSM_state:     ds 1
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
mf:     dbit 1 ; math32 sign
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
; 11 bits used

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

PWM_OUT        EQU P1.3 ; Pin connected to the ssr for outputing pwm signal

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
Temp_Suffix:     db ' degrees Celsius', 0

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
$include(Timer2_ISR.inc) ; Timer 2 ISR for 1ms tick and pwm signal generation
$include(LCD_4bit_DE10Lite_no_RW.inc) ; LCD related functions and utility macros
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
    
    ; ==========================================
    ; PART 1: STATIC TEXT (Title)
    ; Runs ONLY when the state changes
    ; ==========================================
    
    ; [FIX] "Trampoline" logic for long distance jump
    ; If signal is SET (1), we stay here and update.
    ; If signal is CLEAR (0), we Long Jump to the Live Update section.
    jb state_change_signal, Do_Static_Update
    ljmp Check_Live_Update

Do_Static_Update:
    clr state_change_signal
    
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
    ljmp LCD_Update_Temp_Value ; Draw Temp immediately!

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
    ; [FIX] Check distance safe logic for State 7
    cjne a, #7, LCD_Done_Bridge ; If not 7, we are done
    Set_Cursor(1,1)
    Send_Constant_String(#String_state7)
    ljmp LCD_Done_Bridge

; Local bridge to reach the far-away LCD_Done
LCD_Done_Bridge:
    ljmp LCD_Done

; ==========================================
; PART 2: DYNAMIC VALUES (Temperature)
; Runs every time 'one_second_flag' is set
; ==========================================
Check_Live_Update:
    jnb one_second_flag, LCD_Done_Bridge
    clr one_second_flag
    lcall Send_Temp_Serial
    
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

; --- HELPER: Prints "XXX C" on Line 2 ---
LCD_Update_Temp_Value:
    Set_Cursor(2, 1)
    
    ; Convert current_temp to BCD
    mov x, current_temp
    mov x+1, current_temp+1
    mov x+2, current_temp+2
    mov x+3, current_temp+3
    lcall hex2bcd
    
    ; Print Hundreds
    mov a, bcd+1
    anl a, #0x0F
    add a, #0x30
    lcall ?WriteData
    
    ; Print Tens
    mov a, bcd+0
    swap a
    anl a, #0x0F
    add a, #0x30
    lcall ?WriteData
    
    ; Print Ones
    mov a, bcd+0
    anl a, #0x0F
    add a, #0x30
    lcall ?WriteData
    
    ; Print 'C'
    mov a, #'C'
    lcall ?WriteData
    
    ; Clear remaining line space (prevents garbage)
    mov a, #' '
    lcall ?WriteData
    lcall ?WriteData

LCD_Done:
    pop acc
    ret
;---------------------------------------------------------

;---------------------------------------------------------
; Serial Temperature Output (format: T=XX.XX degrees Celsius)
; Uses existing primitives: hex2bcd, putchar, SendString
;---------------------------------------------------------
Send_Temp_Serial:
    push acc
    push dpl
    push dph

    ; Convert current_temp to BCD
    mov x, current_temp
    mov x+1, current_temp+1
    mov x+2, current_temp+2
    mov x+3, current_temp+3
    lcall hex2bcd

    ; Print prefix "T="
    mov a, #'T'
    lcall putchar
    mov a, #'='
    lcall putchar

    ; Print Hundreds
    mov a, bcd+1
    anl a, #0x0F
    add a, #0x30
    lcall putchar
    
    ; Print Tens
    mov a, bcd+0
    swap a
    anl a, #0x0F
    add a, #0x30
    lcall putchar

    ; Print Ones
    mov a, bcd+0
    anl a, #0x0F
    add a, #0x30
    lcall putchar

    ; Print ".00"
    mov a, #'.'
    lcall putchar
    mov a, #'0'
    lcall putchar
    mov a, #'0'
    lcall putchar

    ; Print suffix " degrees Celsius"
    mov dptr, #Temp_Suffix
    lcall SendString

    ; New line (CR/LF) for PuTTY
    mov a, #0x0D
    lcall putchar
    mov a, #0x0A
    lcall putchar

    pop dph
    pop dpl
    pop acc
    ret

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
    
    ; --- 1 Second has passed! ---
    setb one_second_flag
    
    mov a, current_time_sec
    cjne a, #59, IncCurrentTimeSec 
    
    ; --- FIX: 59s -> 0s AND Increment Minute ---
    mov current_time_sec, #0
    inc current_time_minute    ; <--- YOU WERE MISSING THIS!
    ; -------------------------------------------
    
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
; Checks if we have reached the user's target temperatures.
; Sets 'soak_temp_reached' if current_temp >= soak_temp
; Sets 'reflow_temp_reached' if current_temp >= reflow_temp
;-------------------------------------------------------------------------------;
Temp_Compare:
    ; Reset flags initially
    clr soak_temp_reached
    clr reflow_temp_reached

    push acc
    push psw
    push AR0
    push AR1
    push AR2
    
    ; --- 1. CHECK SOAK TEMP ---
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
    jb mf, Check_Reflow_Threshold ; If Current < Target, jump (Flag stays 0)
    
    ; If we are here, Current >= Target
    setb soak_temp_reached

Check_Reflow_Threshold:
    ; --- 2. CHECK REFLOW TEMP ---
    ; Copy current_temp to X (Need to reload X because math32 destroys it)
    mov R0, #current_temp
    mov R1, #x
    lcall Copy4_Bytes_R0_to_R1

    ; Copy reflow_temp to Y
    mov R0, #reflow_temp
    mov R1, #y
    lcall Copy4_Bytes_R0_to_R1

    ; Compare
    lcall x_lt_y
    jb mf, Temp_Compare_Done
    
    ; If Current >= Target
    setb reflow_temp_reached

Temp_Compare_Done:
    pop AR2
    pop AR1
    pop AR0
    pop psw
    pop acc
    ret
;-------------------------------------------------------------------------------;
; Time_Compare
;
; PURPOSE:
;   Compare the elapsed time against soak and reflow
;   time limits.
;
; BEHAVIOR:
;   - If current_time >= soak_time   if soak_time_reached    = 1
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
    jb   mf, Safety_TC_Done        ; still < 60s ? keep waiting

    ; We reached 60s: close the startup window so it won't re-check later
    clr  tc_startup_window

    ; Now check: current_temp < 50 ?
    mov  R0, #current_temp
    mov  R1, #x
    lcall Copy4_Bytes_R0_to_R1

    Load_Y(50)
    lcall x_lt_y
    jnb  mf, Safety_TC_Done        ; temp >= 50 ? pass

    ; FAIL: at 60s, still below 50C ? abort
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
;-------------------------------------------------------------------------------;
; FSM LOGIC (Button Logic Fixed)
;-------------------------------------------------------------------------------;
Control_FSM:
    mov a, Control_FSM_state
    sjmp Control_FSM_state0

Control_FSM_state0_a:
    mov Control_FSM_state, #0
    setb state_change_signal
    
Control_FSM_state0:
    cjne a, #0, Control_FSM_state1
    jb P1.0, Control_FSM_done_bridge ; If Button High (Not Pressed), Exit
    lcall Wait_For_P1_0_Release      ; If Low (Pressed), Wait & Proceed
    sjmp Control_FSM_state1_a  
    
Control_FSM_done_bridge:
    ret

Control_FSM_state1_a:
    inc Control_FSM_state
    mov Current_State, #0
    lcall Update_Screen_Full 
    setb state_change_signal
    mov a, Control_FSM_state
    
Control_FSM_state1:
    cjne a, #1, Control_FSM_state2
    lcall Check_Buttons 
    lcall Check_Keypad
    
    ; FIX: Check if Button is HIGH (Not Pressed). If so, exit.
    jb P1.0, Control_FSM_state1_ret
    
    ; If we get here, Button is LOW (Pressed)
    lcall Wait_For_P1_0_Release
    lcall Update_FSM_Variables
    sjmp Control_FSM_state2_a
Control_FSM_state1_ret:
    ret

; --- STATE 2: RAMP TO SOAK ---
Control_FSM_state2_a:
    inc Control_FSM_state
    setb state_change_signal
Control_FSM_state2:
    cjne a, #2, Control_FSM_state3
    jnb PB2_flag, State2_Check
    clr PB2_flag
    ljmp Control_FSM_state6_a ; Pause
State2_Check:
    jnb soak_temp_reached, State2_Ret
    clr soak_temp_reached
    inc Control_FSM_state
    setb state_change_signal
    mov current_time_sec, #0
    mov current_time_minute, #0
    
    ; --- ADD THIS LINE ---
    clr soak_time_reached  ; Ensure we start fresh!
    ; ---------------------

State2_Ret:
    ret

; --- STATE 3: SOAK PHASE ---
Control_FSM_state3:
    cjne a, #3, Control_FSM_state4
    jnb PB2_flag, State3_Check
    clr PB2_flag
    ljmp Control_FSM_state6_a
State3_Check:
    jnb soak_time_reached, State3_Ret
    clr soak_time_reached
    inc Control_FSM_state      
    setb state_change_signal 
State3_Ret:
    ret

; --- STATE 4: RAMP TO PEAK ---
Control_FSM_state4:
    cjne a, #4, Control_FSM_state5
    jnb PB2_flag, State4_Check
    clr PB2_flag
    ljmp Control_FSM_state6_a
State4_Check:
    jnb reflow_temp_reached, State4_Ret
    clr reflow_temp_reached
    inc Control_FSM_state
    setb state_change_signal
    mov current_time_sec, #0
    mov current_time_minute, #0
    ; --- ADD THIS LINE ---
    clr reflow_time_reached ; Kill the ghost flag
    ; ---------------------
State4_Ret:
    ret

; --- STATE 5: REFLOW PHASE ---
Control_FSM_state5:
    cjne a, #5, Control_FSM_state6_trampoline
    sjmp State5_Logic
Control_FSM_state6_trampoline:
    ljmp Control_FSM_state6

State5_Logic:
    jnb PB2_flag, State5_Check
    clr PB2_flag
    ljmp Control_FSM_state6_a
State5_Check:
    jnb reflow_time_reached, State5_Ret
    clr reflow_time_reached
    ljmp Control_FSM_state6_a
State5_Ret:
    ret

; --- STATE 6: COOLING ---
Control_FSM_state6_a:
    inc Control_FSM_state
    setb state_change_signal
Control_FSM_state6:
    cjne a, #6, Control_FSM_state7
    ; Wait for Cooling Temp Reached
    jnb cooling_temp_reached, State6_Ret
    clr cooling_temp_reached
    inc Control_FSM_state
    setb state_change_signal
State6_Ret:
    ret

; --- STATE 7: DONE ---
Control_FSM_state7:
    cjne a, #7, Control_FSM_done
    
    ; FIX: Check if Button is HIGH (Not Pressed). If so, exit.
    jb PB0_flag, Control_FSM_Reset_Logic ; Wait, PB0_flag is software flag?
    
    ; Let's assume you meant the physical button P1.0 like State 0
    jb P1.0, Control_FSM_done
    
    lcall Wait_For_P1_0_Release
    ljmp Control_FSM_state0_a

Control_FSM_Reset_Logic:
    ; If using PB0_flag from ISR, handle here
    clr PB0_flag
    ljmp Control_FSM_state0_a

Control_FSM_done:
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
    ; 2. THE "DIRTY DELAY" (Fixes Reset Garbage)
    ; We burn ~100ms here using a raw loop. 
    ; We cannot use timers yet because they aren't initialized.
    ; --------------------------------------------------------
    mov R0, #250
Reset_Delay_Outer:
    mov R1, #255
Reset_Delay_Inner:
    djnz R1, Reset_Delay_Inner
    djnz R0, Reset_Delay_Outer
    ; --------------------------------------------------------

    ; ... NOW continue with your normal Port Configuration ...
    
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
    ; Turn off all the LEDs
    mov LEDRA, #0 ; LEDRA is bit addressable
    mov LEDRB, #0 ; LEDRB is NOT bit addresable

    ; Enable Global interrupts
    setb EA  

    ; FSM initial states
    mov KEY1_DEB_state, #0
    mov SEC_FSM_state, #0
    mov Control_FSM_state, #0
    mov Current_State, #0
    
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
    
    setb state_change_signal

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
    
    ; Added to take temp readings
    lcall Read_Thermocouple
    
    ; 1. Check if we reached temp (Observer)
    lcall Temp_Compare
    
    ; 2. Decide heater power based on flags (Driver)
    lcall Power_Control
    
    ; 3. [FIX] Calculate Total Seconds (Minutes * 60 + Seconds)
    ; ---------------------------------------------------------
    ; Load Minutes into X
    mov x+0, current_time_minute
    mov x+1, #0
    mov x+2, #0
    mov x+3, #0
    
    ; Multiply by 60 (Minutes -> Seconds)
    Load_y(60)
    lcall mul32
    
    ; Load Seconds into Y
    mov y+0, current_time_sec
    mov y+1, #0
    mov y+2, #0
    mov y+3, #0
    
    ; Add them together (Total Seconds = X + Y)
    lcall add32
    
    ; Store Final Result into 'current_time'
    mov current_time+0, x+0
    mov current_time+1, x+1
    mov current_time+2, x+2
    mov current_time+3, x+3
    
    lcall Time_Compare
    
    lcall Safety_Check_TC


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
    ; --- FORCE INPUT MODE ---
    ; This clears any '0' the LCD library might have written to our buttons
    orl P0, #055H   ; Sets P0.0, P0.2, P0.4, and P0.6 to '1' (Input Mode)
    ; ------------------------

    jnb BTN_SOAK_TEMP, Btn_Soak_Temp_Press
    jnb BTN_SOAK_TIME, Btn_Soak_Time_Press
    jnb BTN_REFL_TEMP, Btn_Refl_Temp_Press
    jnb BTN_REFL_TIME, Btn_Refl_Time_Press
    ret

Btn_Soak_Temp_Press:
    lcall Wait_25ms_BLOCKING
    mov Current_State, #1
    mov Cursor_Idx, #0
    sjmp Redraw_Screen

Btn_Soak_Time_Press:
    lcall Wait_25ms_BLOCKING
    mov Current_State, #2
    mov Cursor_Idx, #0
    sjmp Redraw_Screen

Btn_Refl_Temp_Press:
    lcall Wait_25ms_BLOCKING
    mov Current_State, #3
    mov Cursor_Idx, #0
    sjmp Redraw_Screen

Btn_Refl_Time_Press:
    lcall Wait_25ms_BLOCKING 
    mov Current_State, #4
    mov Cursor_Idx, #0
    sjmp Redraw_Screen

Redraw_Screen:
    ; Wait for button release
    jnb BTN_SOAK_TEMP, $
    jnb BTN_SOAK_TIME, $
    jnb BTN_REFL_TEMP, $
    jnb BTN_REFL_TIME, $

    lcall Update_Screen_Full
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
    ; 1. Check if we are already waiting
    jb wait25_active, Check_Timer_Status
    
    ; 2. Check if we just finished
    jnb wait25_done, Start_New_Timer
    
    ; 3. Timer is DONE! Reset flags and return True
    clr wait25_done
    setb C          ; Carry = 1 means "Done"
    ret

Start_New_Timer:
    ; 4. Start a new 25ms wait
    mov wait25_count, #0
    setb wait25_active
    clr C           ; Carry = 0 means "Not Done Yet"
    ret

Check_Timer_Status:
    ; 5. Still waiting... return False immediately
    clr C           ; Carry = 0 means "Not Done Yet"
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
    cjne A, #2, RC_Check_State_4  
    sjmp Adjust_Cursor_Time
RC_Check_State_4:             
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

; Blocking wrapper for LCD clear (keeps old behavior just for this)
Wait_25ms_BLOCKING:
    lcall Wait_25ms
    jnc Wait_25ms_BLOCKING ; Keep jumping back until Done (C=1)
    ret

Clear_Screen_Func:
    mov A, #0x01
    lcall ?WriteCommand
    
    ; --- FIX: HARDWARE DELAY LOOP (MAX STRENGTH) ---
    ; The LCD needs ~2ms to clear. 
    ; We use R0=255 to guarantee ~5ms+ delay.
    ; This ensures the LCD is 100% ready before we send "Ramp to Soak".
    mov R0, #255
Clear_Delay_Loop_Outer:
    mov R1, #255
Clear_Delay_Loop_Inner:
    djnz R1, Clear_Delay_Loop_Inner
    djnz R0, Clear_Delay_Loop_Outer
    ; -----------------------------------------------

    mov A, #0x0C  ; Display ON, Cursor OFF
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
    
    
; ================================================================
; MODULE: THERMOCOUPLE ADC DRIVER
; ================================================================
Read_Thermocouple:
    ; 1. Check Non-Blocking Timer (Run once every 25ms)
    lcall Wait_25ms
    jnc Read_TC_Exit ; If 25ms hasn't passed, exit immediately
    
    ; --- 25ms Passed! Time to Read ---
    
    ; 2. Initialize / Trigger ADC
    ; Writing to ADC_C (0xA1) triggers the conversion
    mov ADC_C, #0x80    ; Reset / Strobe
    nop
    nop
    mov ADC_C, #0x00    ; Select Channel 0 (ADCINPUT 0) and Start
    
    ; 3. [FIX] Settle Delay
    ; The DE10-Lite ADC bridge needs time to fetch data from the MAX10 chip.
    ; We burn ~500 cycles to be absolutely safe.
    mov R5, #250
ADC_Settle_Loop:
    nop
    nop
    djnz R5, ADC_Settle_Loop
    
    ; 4. Read Raw Data
    mov x+0, ADC_L      ; Read Low Byte (0xA2)
    mov x+1, ADC_H      ; Read High Byte (0xA3)
    mov x+2, #0
    mov x+3, #0
    
    ; 5. [FIX] Mask the 12-bit Data
    ; The ADC is 12-bit. We MUST zero out the upper 4 bits of the High Byte
    ; or the math below will overflow and return 0.
    mov a, x+1
    anl a, #0x0F
    mov x+1, a
    
    ; 6. Convert to Voltage (Count * 5000 / 4095)
    Load_y(5000)        ; Vref = 5000mV
    lcall mul32         
    
    Load_y(4095)        ; 12-bit resolution
    lcall div32         
    
    ; 7. Convert to Temp (Voltage / 10mV) -> e.g. 250mV / 10 = 25C
    ; Change this Load_y value if your amp gain is different!
    Load_y(10)          
    lcall div32         
    
    ; 8. Store Final Result
    mov current_temp+0, x+0
    mov current_temp+1, x+1
    mov current_temp+2, x+2
    mov current_temp+3, x+3

Read_TC_Exit:
    ret
    
; ================================================================
; MODULE: POWER CONTROLLER (The Brain)
; ================================================================
Power_Control:
    ; Default: Turn Heat OFF (Safety)
    mov power_output+0, #0
    mov power_output+1, #0
    mov power_output+2, #0
    mov power_output+3, #0

    mov a, Control_FSM_state

    ; --- State 2: RAMP TO SOAK ---
    cjne a, #2, PC_Check_Soak
    ; Mode: Full Speed Ahead
    sjmp Set_Max_Power

PC_Check_Soak:
    ; --- State 3: SOAK PHASE ---
    cjne a, #3, PC_Check_Ramp_Reflow
    ; Mode: Maintenance (Low Power)
    ; If Temp > Target, OFF. If Temp < Target, 20% Power.
    jb soak_temp_reached, PC_Done ; If hot enough, stay OFF
    sjmp Set_20_Percent_Power     ; If cold, use 20%

PC_Check_Ramp_Reflow:
    ; --- State 4: RAMP TO REFLOW ---
    cjne a, #4, PC_Check_Reflow
    ; Mode: Full Speed Ahead
    sjmp Set_Max_Power

PC_Check_Reflow:
    ; --- State 5: REFLOW PHASE ---
    cjne a, #5, PC_Done
    ; Mode: Maintenance (Low Power)
    jb reflow_temp_reached, PC_Done
    sjmp Set_20_Percent_Power

PC_Done:
    ret

; --- Power Helpers ---

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

END
