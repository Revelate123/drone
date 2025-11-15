
; timer.inc

jmp end_timer


; INIT_SIMULATION_TIMER
; Configures Timer1 for 0.5-second simulation ticks (2 Hz)
; Uses CTC mode with OCR1A compare match
init_simulation_timer:
init_simulation_timer_prologue:
    push r16
    
init_simulation_timer_body:
    cli
    
    ; Set Timer1 to CTC mode
    ldi r16, 0b00000000
    sts TCCR1A, r16               ; WGM11:10 = 00
    
    ldi r16, 0b00001101           ; WGM13:12 = 01, CS12:10 = 101 (prescaler 1024)
    sts TCCR1B, r16
    
    ; OCR1A = (0.5 * 16,000,000) / 1024 - 1 = 7812 = 0x1E84
    ldi r16, high(7812)
    sts OCR1AH, r16
    ldi r16, low(7812)
    sts OCR1AL, r16
    
    ; Enable Timer1 Compare Match A interrupt
    ldi r16, (1 << OCIE1A)
    sts TIMSK1, r16
    
    ; Clear timer counter
    ldi r16, 0
    sts TCNT1H, r16
    sts TCNT1L, r16
    
    sei
    
init_simulation_timer_epilogue:
    pop r16
    ret


; TIMER1_COMPA ISR
; Triggered every 0.5 seconds (2 Hz)
TIMER1_COMPA:
TIMER1_COMPA_prologue:
    push r16
    in r16, SREG
    push r16
    push YL
    push YH
    
TIMER1_COMPA_body:
    ; Increment 16-bit simulation_counter
    ldi YL, low(simulation_counter)
    ldi YH, high(simulation_counter)
    
    ld r16, Y
    inc r16
    st Y+, r16
    
    brne skip_inc_high
    
    ld r16, Y
    inc r16
    st Y, r16
    
skip_inc_high:
    ; Set flag for main loop
    ldi r16, 1
    sts simulation_tick_flag, r16
    
TIMER1_COMPA_epilogue:
    pop YH
    pop YL
    pop r16
    out SREG, r16
    pop r16
    reti

end_timer: