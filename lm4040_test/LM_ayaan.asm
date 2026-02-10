; ==============================================================================
; FINAL FIX: LM335 + PROBE + LCD + SLOW REFRESH
; ==============================================================================
$NOLIST
$include(MODMAX10)
$LIST

; ------------------------------------------------------------------------------
; 1. PIN DEFINITIONS
; ------------------------------------------------------------------------------
ELCD_RS equ P1.7
ELCD_E  equ P1.1
ELCD_D4 equ P0.7
ELCD_D5 equ P0.5
ELCD_D6 equ P0.3
ELCD_D7 equ P0.1

; ---------------- constants ----------------
FREQ      EQU 33333333
BAUD      EQU 57600
T2LOAD    EQU 65536-(FREQ/(32*BAUD))

ADC_CH_TC     EQU 0           
ADC_CH_REF    EQU 4           
ADC_CH_LM335  EQU 5           
AVG_N         EQU 8           
ADC_FS_mVREF  EQU 4096        

; ---------------- variables ----------------
dseg at 30h
x:      ds 4
y:      ds 4
bcd:    ds 5

dseg at 40h
sample_count:    ds 1
avg_counts:      ds 4
ref_counts:      ds 4
ref_countdown:   ds 1
tc_delta:        ds 4

bseg
mf:     dbit 1

cseg
; ------------------------------------------------------------------------------
; 2. RESET VECTOR
; ------------------------------------------------------------------------------
org 0x0000
    ljmp mycode

; ------------------------------------------------------------------------------
; 3. LIBRARIES
; ------------------------------------------------------------------------------
org 0x0100
$include(math32.asm)
$include(LCD_4bit_DE10Lite_no_RW.inc)

; ==================================================
; ================= LOOKUP TABLE ===================
; ==================================================
myLUT:
    DB 0xC0, 0xF9, 0xA4, 0xB0, 0x99
    DB 0x92, 0x82, 0xF8, 0x80, 0x90
    DB 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E

; ==================================================
; ================= SERIAL INIT ====================
; ==================================================
InitSerialPort:
    clr TR2
    mov T2CON, #30H
    mov RCAP2H, #high(T2LOAD)
    mov RCAP2L, #low(T2LOAD)
    setb TR2
    mov SCON, #52H
    setb TI
    ret

putchar:
    jnb TI, putchar
    clr TI
    mov SBUF, a
    ret

; ==================================================
; ================= ADC HELPERS ====================
; ==================================================
ADC_Settle:
    mov R7, #250
AS_1:
    nop
    djnz R7, AS_1
    ret

ReadADC5_to_y: ; Read LM335
    mov ADC_C, #ADC_CH_LM335
    lcall ADC_Settle
    mov y+3, #0
    mov y+2, #0
    mov y+1, ADC_H
    mov y+0, ADC_L
    mov a, y+1
    anl a, #0x0F
    mov y+1, a
    ret

ReadADC4_to_y: ; Read Reference
    mov ADC_C, #ADC_CH_REF
    lcall ADC_Settle
    mov y+3, #0
    mov y+2, #0
    mov y+1, ADC_H
    mov y+0, ADC_L
    mov a, y+1
    anl a, #0x0F
    mov y+1, a
    ret

; ==================================================
; ======= GET ROOM TEMP (LM335) SUBROUTINE =========
; ==================================================
Get_LM335_TempC:
    ; 1. Sum LM335 Samples
    mov x+0,#0
    mov x+1,#0
    mov x+2,#0
    mov x+3,#0
    mov sample_count,#0
LM_SumLoop:
    lcall ReadADC5_to_y
    lcall add32
    inc sample_count
    mov a, sample_count
    cjne a, #AVG_N, LM_SumLoop

    ; Average LM335
    Load_y(AVG_N)
    lcall div32
    mov avg_counts+0, x+0
    mov avg_counts+1, x+1
    mov avg_counts+2, x+2
    mov avg_counts+3, x+3

    ; 2. Sum Reference Samples
    mov x+0,#0
    mov x+1,#0
    mov x+2,#0
    mov x+3,#0
    mov ref_countdown, #AVG_N
REF_SumLoop:
    lcall ReadADC4_to_y
    lcall add32
    djnz ref_countdown, REF_SumLoop

    ; Average Reference
    Load_y(AVG_N)
    lcall div32
    
    ; Check for Divide by Zero
    mov a, x+0
    orl a, x+1
    orl a, x+2
    orl a, x+3
    jnz REF_OK
    Load_x(1) ; Prevent crash
REF_OK:
    mov ref_counts+0, x+0
    mov ref_counts+1, x+1
    mov ref_counts+2, x+2
    mov ref_counts+3, x+3

    ; 3. Math: mV = (Counts * 4096) / Ref_Counts
    mov x+0, avg_counts+0
    mov x+1, avg_counts+1
    mov x+2, avg_counts+2
    mov x+3, avg_counts+3
    
    Load_y(ADC_FS_mVREF)
    lcall mul32
    
    mov y+0, ref_counts+0
    mov y+1, ref_counts+1
    mov y+2, ref_counts+2
    mov y+3, ref_counts+3
    lcall div32

    ; 4. Math: TempC = ((mV * 10) - 27315) / 100
    Load_y(10)
    lcall mul32
    Load_y(27315)
    lcall sub32
    Load_y(100)
    lcall div32
    ret

; ==================================================
; ================= DISPLAY LOGIC ==================
; ==================================================
Display_Voltage_7seg:
    mov dptr, #myLUT
    
    ; HEX3: 0.
    mov a, #0
    movc a, @a+dptr
    anl a, #0x7F       ; Add decimal point
    mov HEX3, a
    
    ; HEX2: Hundreds
    mov a, bcd+1
    anl a, #0x0F
    movc a, @a+dptr
    mov HEX2, a
    
    ; HEX1: Tens
    mov a, bcd+0
    swap a
    anl a, #0x0F
    movc a, @a+dptr
    mov HEX1, a
    
    ; HEX0: Ones
    mov a, bcd+0
    anl a, #0x0F
    movc a, @a+dptr
    mov HEX0, a
    ret

; --- [FIX] ADDED LCD FUNCTION BACK ---
Display_Voltage_LCD:
    Set_Cursor(2,1)
    mov a, #'T'
    lcall ?WriteData
    mov a, #'='
    lcall ?WriteData
    
    ; Print "0."
    mov a, #'0'
    lcall ?WriteData
    mov a, #'.'
    lcall ?WriteData
    
    ; Hundreds
    mov a, bcd+1
    anl a, #0x0F
    orl a, #'0'
    lcall ?WriteData
    
    ; Tens
    mov a, bcd+0
    swap a
    anl a, #0x0F
    orl a, #'0'
    lcall ?WriteData
    
    ; Ones
    mov a, bcd+0
    anl a, #0x0F
    orl a, #'0'
    lcall ?WriteData
    ret

; ==================================================
; ================= MAIN PROGRAM ===================
; ==================================================
mycode:
    clr EA              ; Disable Interrupts (Safety)
    mov SP, #0xC0       ; Move Stack to Upper RAM
    
    mov P0MOD, #0xAA    
    mov P1MOD, #0x82   

    lcall InitSerialPort
    lcall ELCD_4BIT     ; Init LCD

forever:
    ; -------------------------------------------
    ; STEP 1: READ THERMOCOUPLE (WITH AVERAGING)
    ; -------------------------------------------
    mov x+0, #0
    mov x+1, #0
    mov x+2, #0
    mov x+3, #0
    mov R4, #32        

TC_Avg_Loop:
    mov ADC_C, #ADC_CH_TC  
    lcall ADC_Settle       
    
    mov y+0, ADC_L
    mov y+1, ADC_H
    mov y+2, #0
    mov y+3, #0
    
    mov a, y+1
    anl a, #0x0F
    mov y+1, a
    
    lcall add32         
    djnz R4, TC_Avg_Loop
    
    Load_y(32)
    lcall div32
    
    ; -------------------------------------------
    ; STEP 2: CALCULATE PROBE DELTA
    ; -------------------------------------------
    Load_y(5000)
    lcall mul32
    Load_y(4095)
    lcall div32
    
    ; NOTE: Check this gain (12300 divisor) if temp is wrong!
    Load_y(1000)
    lcall mul32
    Load_y(12300)
    lcall div32
    
    mov tc_delta+0, x+0
    mov tc_delta+1, x+1
    mov tc_delta+2, x+2
    mov tc_delta+3, x+3
    
    ; -------------------------------------------
    ; STEP 3: GET ROOM TEMP (LM335)
    ; -------------------------------------------
    lcall Get_LM335_TempC
    
    ; -------------------------------------------
    ; STEP 4: ADD THEM UP
    ; -------------------------------------------
    mov y+0, tc_delta+0
    mov y+1, tc_delta+1
    mov y+2, tc_delta+2
    mov y+3, tc_delta+3
    lcall add32
    
    ; -------------------------------------------
    ; STEP 5: DISPLAY
    ; -------------------------------------------
    lcall hex2bcd
    lcall Display_Voltage_7seg
    lcall Display_Voltage_LCD  ; [FIX] Call the LCD function!
    
    ; --- [FIX] SLOW DOWN THE REFRESH RATE ---
    ; Old delay was ~0.5ms. This delay is ~200ms.
    ; This stops the "flickering/spazzing".
    mov R5, #10
Delay_L3:
    mov R6, #250
Delay_L2:
    mov R7, #250
Delay_L1:
    djnz R7, Delay_L1
    djnz R6, Delay_L2
    djnz R5, Delay_L3
    
    ljmp forever

END