;-------------------------------------------------------------------------------
; Timer2_ISR.inc
; Contains Initialization and ISR for the 1ms System Timer
;-------------------------------------------------------------------------------

;-------------------------------------------------------------------------------
; Routine to initialize the ISR for timer 2
;-------------------------------------------------------------------------------
Timer2_Init:
    mov T2CON, #0 ; Stop timer/counter.  Autoreload mode.
    mov TH2, #high(TIMER2_RELOAD)
    mov TL2, #low(TIMER2_RELOAD)
    ; Set the reload value
    mov RCAP2H, #high(TIMER2_RELOAD)
    mov RCAP2L, #low(TIMER2_RELOAD)
    
    clr TF2       ; Clear flag just in case
    ; Enable the timer and interrupts
    setb ET2      ; Enable timer 2 interrupt
    setb TR2      ; Enable timer 2
    ret

;-------------------------------------------------------------------------------
; ISR for timer 2.  Runs every 1 ms
;-------------------------------------------------------------------------------
Timer2_ISR:
    push acc
    push psw
    
    clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in ISR

    ; --- 1. Existing FSM Timers ---
    inc KEY1_DEB_timer
    inc SEC_FSM_timer
    setb one_ms_pwm_flag 

    ; --- 2. NEW: Non-Blocking Delay Counters ---
    
    ; A. BUTTON DELAY
    jnb wait25_btn_active, T2_Check_Keypad
    inc wait25_btn_cnt
    mov a, wait25_btn_cnt
    cjne a, #25, T2_Check_Keypad
    setb wait25_btn_done
    clr wait25_btn_active
    
    ; B. KEYPAD DELAY
T2_Check_Keypad:
    jnb wait25_keypad_active, T2_Check_ADC
    inc wait25_keypad_cnt
    mov a, wait25_keypad_cnt
    cjne a, #25, T2_Check_ADC
    setb wait25_keypad_done
    clr wait25_keypad_active

    ; C. ADC DELAY (Thermocouple)
T2_Check_ADC:
    jnb wait25_adc_active, T2_Check_LCD
    inc wait25_adc_cnt
    mov a, wait25_adc_cnt
    cjne a, #25, T2_Check_LCD
    setb wait25_adc_done
    clr wait25_adc_active

    ; D. LCD DELAY
T2_Check_LCD:
    jnb wait25_lcd_active, T2_Check_Generic
    inc wait25_lcd_cnt
    mov a, wait25_lcd_cnt
    cjne a, #25, T2_Check_Generic
    setb wait25_lcd_done
    clr wait25_lcd_active

    ; E. GENERIC DELAY (Used by Wait_25ms in Main)
T2_Check_Generic:
    jnb wait25_active, Timer2_ISR_done
    inc wait25_count
    mov a, wait25_count
    cjne a, #25, Timer2_ISR_done
    setb wait25_done      ; Tells Wait_25ms that we are finished
    clr wait25_active     ; Stop counting

Timer2_ISR_done:
    pop psw
    pop acc
    reti
	
