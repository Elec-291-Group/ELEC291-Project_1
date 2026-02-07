$NOLIST
$MODN76E003
$LIST

;  N76E003 pinout:
;                               -------
;       PWM2/IC6/T0/AIN4/P0.5 -|1    20|- P0.4/AIN5/STADC/PWM3/IC3
;               TXD/AIN3/P0.6 -|2    19|- P0.3/PWM5/IC5/AIN6
;               RXD/AIN2/P0.7 -|3    18|- P0.2/ICPCK/OCDCK/RXD_1/[SCL]
;                    RST/P2.0 -|4    17|- P0.1/PWM4/IC4/MISO
;        INT0/OSCIN/AIN1/P3.0 -|5    16|- P0.0/PWM3/IC3/MOSI/T1
;              INT1/AIN0/P1.7 -|6    15|- P1.0/PWM2/IC2/SPCLK
;                         GND -|7    14|- P1.1/PWM1/IC1/AIN7/CLO
;[SDA]/TXD_1/ICPDA/OCDDA/P1.6 -|8    13|- P1.2/PWM0/IC0
;                         VDD -|9    12|- P1.3/SCL/[STADC]
;            PWM5/IC7/SS/P1.5 -|10   11|- P1.4/SDA/FB/PWM1
;                               -------
;
Z8     MACRO
    DB 0,0,0,0,0,0,0,0
ENDM

Z16    MACRO
    Z8
    Z8
ENDM

Z32    MACRO
    Z16
    Z16
ENDM

Z64    MACRO
    Z32
    Z32
ENDM

Z128   MACRO
    Z64
    Z64
ENDM

Z200   MACRO
    Z128
    Z64
    Z8
ENDM

CLK               EQU 16600000 ; Microcontroller system frequency in Hz
BAUD              EQU 115200 ; Baud rate of UART in bps
TIMER1_RELOAD     EQU (0x100-(CLK/(16*BAUD)))
TIMER0_RELOAD_1MS EQU (0x10000-(CLK/1000))
	
ORG 0x0000
	ljmp main

;                     1234567890123456    <- This helps determine the location of the counter
test_message:     db '* TEMPERATURE *', 0
value_message:    db 'T(pin 14)=      ', 0

CSEG
; These 'equ' must match the hardware wiring
LCD_RS equ P1.3
LCD_E  equ P1.4
LCD_D4 equ P0.0
LCD_D5 equ P0.1
LCD_D6 equ P0.2
LCD_D7 equ P0.3

$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$LIST

; code segment reserved block
CMD_T_NUM EQU 10

CMD_T_ELE_SIZE  EQU  9
CT_NAME_ELE_SIZE  EQU  8
CT_HELP_ELE_INFO_SIZE  EQU  20
CT_USAGE_ELE_SIZE  EQU  20

CMD_T_SIZE EQU (CMD_T_NUM * CMD_T_SIZE) ; 90
CT_NAME_SIZE EQU (CMD_T_NUM * CT_NAME_SIZE) ; 80
CT_HELP_INFO_SIZE EQU (CMD_T_NUM * CT_HELP_INFO_SIZE) ; 200
CT_USAGE_SIZE EQU (CMD_T_NUM * CT_USAGE_SIZE) ; 200

TOTAL_SIZE EQU  (CMD_T_SIZE + CT_NAME_SIZE + CT_HELP_INFO_SIZE + CT_USAGE_SIZE) ; 560

;PUBLIC  CODE_BLOCK_START
;PUBLIC  CMD_T_START, CMD_T_END
;PUBLIC  CT_NAME_START, CT_NAME_END
;PUBLIC  CT_HELP_INFO_START, CT_HELP_INFO_END
;PUBLIC  CT_USAGE_START, CT_USAGE_END
;PUBLIC  CODE_BLOCK_END

CODE_BLOCK_START:

CMD_T_START: 
	ZERO_FILL CMD_T_SIZE
CMD_T_END:

CT_NAME_START:
	"lcd"
CT_NAME_END:

CT_HELP_INFO_START: 
	"lcd <on/off>"
CT_HELP_INFO_END:

CT_USAGE_START: 
	"Close or Open LCD"
CT_USAGE_END:

CODE_BLOCK_END:

; Data Segment Definitions
	DSEG AT 30H
CLI_ARGC: ds 1
CLI_ARGV: ds 10 ; 10 args * 1 bytes
CLI_LINE_BUF: ds 128
CLI_LINE_SIZE equ 128

PARSE_PTRL: ds 2
PARSE_STATE: ds 1
IN_QUOTE equ 0

CURRENT_PERM: ds 1

TEMP_R0: ds 1
TEMP_R1: ds 1
TEMP_DPTR: ds 2

; math.inc
x:   ds 4 ;byte
y:   ds 4
bcd: ds 5
VAL_LM4040: ds 2

; Bit Definitions
	BSEG
QUOTE_FLAG: dbit 1
CMD_FOUND: dbit 1

; Constants
CLI_MAX_ARGS equ 10
SPACE_CHAR equ 20h
TAB_CHAR equ 09h
QUOTE_CHAR equ 22h
NULL_CHAR equ 00h

$NOLIST
$include(math32.inc)
$LIST
	
	
CSEG
Init_All:
	; Configure all the pins for biderectional I/O
	mov	P3M1, #0x00
	mov	P3M2, #0x00
	mov	P1M1, #0x00
	mov	P1M2, #0x00
	mov	P0M1, #0x00
	mov	P0M2, #0x00
	
	orl	CKCON, #0x10 ; CLK is the input for timer 1
	orl	PCON, #0x80 ; Bit SMOD=1, double baud rate
	mov	SCON, #0x52
	anl	T3CON, #0b11011111
	anl	TMOD, #0x0F ; Clear the configuration bits for timer 1
	orl	TMOD, #0x20 ; Timer 1 Mode 2
	mov	TH1, #TIMER1_RELOAD ; TH1=TIMER1_RELOAD;
	setb TR1
	
	; Using timer 0 for delay functions.  Initialize here:
	clr	TR0 ; Stop timer 0
	orl	CKCON,#0x08 ; CLK is the input for timer 0
	anl	TMOD,#0xF0 ; Clear the configuration bits for timer 0
	orl	TMOD,#0x01 ; Timer 0 in Mode 1: 16-bit timer
	
	;----------------------------------------------------------------;
	; Initialize the pins used by the ADC (P1.1, P1.7) as input.
	orl	P1M1, #0b10000010
	anl	P1M2, #0b01111101
	
	; Initialize and start the ADC:
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07
	
	; AINDIDS select if some pins are analog inputs or digital I/O:
	; AINDIDS disable digital output circuit
	mov AINDIDS, #0x00 ; Disable all analog inputs
	orl AINDIDS, #0b10000001 ; Activate AIN0 and AIN7 analog inputs
	
	; ADCEN active adc circuit
	orl ADCCON1, #0x01 ; Enable ADC
	;----------------------------------------------------------------;
	
	; result: ADCRH high8, ADCRL low4
	ret
	
;------------;
;UART0_init  ;
;------------;
InitSerialPort:
    ; Since the reset button bounces, we need to wait a bit before
    ; sending messages, otherwise we risk displaying gibberish!
    mov R1, #200
    mov R0, #104
    djnz R0, $   ; 4 cycles->4*60.285ns*104=25us
    djnz R1, $-4 ; 25us*200=5.0ms

    ; Now we can proceed with the configuration of the serial port
	orl	CKCON, #0x10 ; CLK is the input for timer 1
	orl	PCON, #0x80 ; Bit SMOD=1, double baud rate
	mov	SCON, #0x52
	anl	T3CON, #0b11011111
	anl	TMOD, #0x0F ; Clear the configuration bits for timer 1
	orl	TMOD, #0x20 ; Timer 1 Mode 2
	mov	TH1, #TIMER1_RELOAD
	setb TR1
    ret
	
wait_1ms:
	clr	TR0 ; Stop timer 0
	clr	TF0 ; Clear overflow flag
	mov	TH0, #high(TIMER0_RELOAD_1MS)
	mov	TL0,#low(TIMER0_RELOAD_1MS)
	setb TR0
	jnb	TF0, $ ; Wait for overflow
	ret

; Wait the number of miliseconds in R2
waitms:
	lcall wait_1ms
	djnz R2, waitms
	ret
	
; CLI system
CLI_SetPermission:
	mov CURRENT_PERM, a
	ret
	
; Parse input command line 	
; DPTR input string
; R0 pointer to CLI line buffer
; R1 index of argv
; R2 index of CLI line buffer
CLI_Parse:
	push DPH
	push DPL
	
	mov R0, #CLI_LINE_BUF
	mov R2, #CLI_LINE_SIZE - 1
	
CLI_Parse_Copy:
	clr a
	movc a, @A+DPTR
	mov @R0, a
	jz CLI_Parse_CopyDone
	inc DPTR
	inc R0
	djnz R2, CLI_Parse_Copy
	
CLI_Parse_CopyDone:
	mov @R0, #NULL_CHAR
	
	mov CLI_ARGC, #0
	mov R0, #CLI_LINE_BUF
	mov R1, #0  ; argv index
	clr QUOTE_FLAG
	
CLI_Parse_Loop:

CLI_Parse_SkipWS:
	mov A, @R0
	jz CLI_Parse_Done
	cjne a, #SPACE_CHAR, CLI_Parse_ChkTab
	sjmp CLI_Parse_SkipWS
	
CLI_Parse_ChkTab:
	cjne a, #TAB_CHAR, CLI_Parse_NotWS
CLI_Parse_NextWS:
	inc R0
	sjmp CLI_Parse_SkipWS
	
CLI_Parse_NotWS:
	mov a, CLI_ARGC
	cjne a, #CLI_MAX_ARGC, CLI_Parse_NotMax
	sjmp CLI_Parse_Done
	
CLI_Parse_NotMax:
	mov a, @R0
	cjne a, #QUOTE_CHAR, CLI_Parse_NoQuote
	
	setb QUOTE_FLAG
	inc R0
	
	mov a, R1
	add a, #CLI_ARGV
	; R2 pointer to CLI argv list
	mov R2, a
	mov a, R0
	mov @R2, a
	
CLI_Parse_FindQuote:
	mov a, @R0 ; char itself 
	jz CLI_Parse_QuoteEnd
	cjne a, #QUOTE_CHAR, CLI_Parse_QuoteNext
	inc R0
	mov @R0, #NULL_CHAR
	sjmp CLI_Parse_ArgDone
	
CLI_Parse_QuoteNext:
	inc R0
	sjmp CLI_Parse_FindQuote
	
CLI_Parse_QuoteEnd:
	sjmp CLI_Parse_ArgDone
	
CLI_Parse_NoQuote:
	mov a, R1
	add a, #CLI_ARGV
	mov R2, a
	mov a, R0
	mov @R2, a
	
CLI_Parse_FindEnd:
	mov a, @R0
	jz CLI_Parse_ArgDone
	cjne a, #SPACE_CHAR, CLI_Parse_ChkTab2
	sjmp CLI_Parse_EndFound
CLI_Parse_ChkTab2:
	cjne a, #TAB_CHAR, CLI_Parse_NotEnd
	
CLI_Parse_EndFound:
	mov @R0, #NULL_CHAR
	inc R0
	sjmp CLI_Parse_ArgDone
	
CLI_Parse_NotEnd:
	inc R0
	sjmp CLI_Parse_FindEnd
	
CLI_Parse_ArgDone:
	inc CLI_ARGC
	inc R1
	sjmp CLI_Parse_Loop
	
CLI_Parse_Done:
	pop DPL
	pop DPH
	mov a, CLI_ARGC
	ret
	
; find out which command match
;
;

CMD_STRUCT_SIZE equ 9
CMD_NUM equ 2

FindCmd: 
	mov R0, A
	mov DPTR, #CMD_T_START
	
	mov R7, #CMD_NUM
	
NextStruct:
	mov R2, DPL
	mov R3, DPH
	
	clr a
	movc a, @A+DPTR
	mov R4, a
	inc DPTR
	clr a
	movc a, @A+DPTR
	mov R5, A
	
	mov DPL, R4
    mov DPH, R5
    
    mov a, R0
    mov R1, a ; notice
    
CompareLoop:
	clr a
	movc a, @A+DPTR
	mov R6, a
	inc DPTR
	
	mov A, @R1
	inc R1
	
	cjne A, R6, NotMatch
	
	jz Matched
	
	sjmp CompareLoop
	
NotMatch:
    mov   DPL, R2
    mov   DPH, R3

    mov   A, DPL
    add   A, #CMD_STRUCT_SIZE
    mov   DPL, A
    mov   A, DPH
    addc  A, #0
    mov   DPH, A

    djnz  R7, NextStruct

    ; Not found
    clr   C
    ret

Matched:
    ; Return struct address saved in R2:R3
    mov   DPL, R2
    mov   DPH, R3
    setb  C
    setb CMD_FOUND
    ret

;
; input: DPTR point to input string
; CLI_Process
CLI_Process:
	clr a 
	movc a, @A+DPTR
	jnz CLI_Proc_NotEmpty
	
	mov A, #0
	ret
	
CLI_Proc_NotEmpty:
	lcall CLI_Parse
	
	mov a, CLI_ARGC
	jnz CLI_Proc_HaveArgs
	mov a, #0
	ret
	
CLI_Proc_HaveArgs:
	mov a, #CLI_ARGV
	lcall FindCmd
	
	mov a, CMD_FOUND
	cjne a, #0, CLI_Proc_CmdFound
	lcall Printf_UnknowCmd
	mov a, #0FFh
	ret
	
CLI_Proc_CmdFound:
	push DPH
	push DPL
	
	mov a, DPL
	add a, #8
	mov DPL, a
	mov a, DPH
	addc a, #0
	mov DPH, a
	
	clr a
	movc a, @A+DPTR
	mov R7, a ; R7 perm
	
	mov a, CURRENT_PERM
	clr c
	subb a, R7
	jnc CLI_Proc_PermOK
	
	pop DPL
	pop DPH
	lcall Printf_PermDenied
	mov a, #0FEh
	ret
	
CLI_Proc_PermOK:
	pop DPL
	pop DPH
	
	mov a, DPL
	add a, #6
	mov DPL, a
	mov a, DPH
	addc a, #0
	mov DPH, a
	
	clr a
	movc a, @A+DPTR
	mov R2, a
	inc DPTR
	clr a
	movc a, @A+DPTR
	mov R3, a
	
	mov DPL, R2
	mov DPH, R3
	mov a, CLI_ARGC
	mov R7, a
	mov 
	
	
; normal part

; We can display a number any way we want.  In this case with
; four decimal places.
Display_formated_BCD:
	Set_Cursor(2, 10)
	Display_BCD(bcd+2)
	Display_char(#'.')
	Display_BCD(bcd+1)
	Display_BCD(bcd+0)
	Set_Cursor(2, 10)
	Display_char(#'=')
	ret

Read_ADC:
	clr ADCF
	setb ADCS ;  ADC start trigger signal
    jnb ADCF, $ ; Wait for conversion complete
    
    ; Read the ADC result and store in [R1, R0]
    mov a, ADCRL
    anl a, #0x0f
    mov R0, a
    mov a, ADCRH   
    swap a
    push acc
    anl a, #0x0f
    mov R1, a
    pop acc
    anl a, #0xf0
    orl a, R0
    mov R0, A
	ret
	
;-------------------;
; A = received bit  ;
; A= bit to send    ;
;-------------------;
UART0_GetChar:
WaitRI:
	jnb RI, WaitRI
	clr RI
	mov A, SBUF
	ret
	
UART0_PutChar:
	clr TI
	mov SBUF, A
	
WaitTI:
	jnb TI, WaitTI
	clr TI
	ret
	
;-------------------------------;
; Put a line of char into RAM   ;
;-------------------------------;
GetLine128_UART:
	push ACC
	push AR0
	push AR1
	push AR2
	
	mov R0, #CLI_LINE_BUF
	mov R1, #128
	
ReadLoop:
	lcall UART0_GetChar
	mov R2, a
	lcall UART0_PutChar
	
	cjne a, #NULL_CHAR, NotNULL
	sjmp NULLLoop
	
NotNULL:
	mov @R0, R2
	inc R0
	dec R1
	djnz R1, ReadLoop
	sjmp DoneLine
	
NULLLoop:
	mov a, R1
	jz DoneLine
	mov a, #NULL_CHAR
	mov @R0, a
	inc R0
	djnz R1, PadLoop
	
DoneLine:
	pop AR2
	pop AR1
	pop AR0
	pop ACC
	ret
	
	
main:
	mov sp, #0x7f
	lcall Init_All
	lcall InitSerialPort
    lcall LCD_4BIT
    
    ; initial messages in LCD
	Set_Cursor(1, 1)
    Send_Constant_String(#test_message)
	Set_Cursor(2, 1)
    Send_Constant_String(#value_message)
    
Forever:

	; Read the 2.08V LM4040 voltage connected to AIN0 on pin 6
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x00 ; Select channel 0
	lcall Read_ADC
	; Save result for later use
	mov VAL_LM4040+0, R0
	mov VAL_LM4040+1, R1

	; Read the signal connected to AIN7
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	lcall Read_ADC
    
    ; Convert to voltage
	mov x+0, R0
	mov x+1, R1
	; Pad other bits with zero
	mov x+2, #0
	mov x+3, #0
	Load_y(40959) ; The MEASURED voltage reference: 4.0959V, with 4 decimal places
	lcall mul32
	; Retrive the ADC LM4040 value
	mov y+0, VAL_LM4040+0
	mov y+1, VAL_LM4040+1
	; Pad other bits with zero
	mov y+2, #0
	mov y+3, #0
	lcall div32
	
	Load_y(27300)
	lcall sub32
	
	mov y+0, #100d
	mov y+1, #0x00
	mov y+2, #0x00
	mov y+3, #0x00
	lcall mul32

	; Convert to BCD and display
	lcall hex2bcd
	lcall Display_formated_BCD
	
	; Wait 500 ms between conversions
	mov R2, #250
	lcall waitms
	mov R2, #250
	lcall waitms
	
	ljmp Forever
END
