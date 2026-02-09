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

	setb one_ms_pwm_flag ; set the one millisecond flag for pwm signal generation
	setb one_millisecond_flag_servo ; set the one millsiecond flag for servo pwm signal generation

Timer2_ISR_done:
	pop psw
	pop acc
	reti
;-----------------------------------------------------------------------------------------------;
