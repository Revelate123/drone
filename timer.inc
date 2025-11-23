
; timer.inc

jmp end_timer


; INIT_SIMULATION_TIMER
; Configures Timer1 for 0.1-second simulation ticks (10 Hz)
; Uses CTC mode with OCR1A compare match
init_simulation_timer:
init_simulation_timer_prologue:
    push r16
    
init_simulation_timer_body:
    cli
    
    ; Set Timer1 to CTC mode
    ldi r16, 0b00000000
    sts TCCR1A, r16               ; WGM11:10 = 00
    
    ; CHANGED: Prescaler from 1024 to 256
	; 16,000,000/ 256 = 62500Hz
	; 1/62500 = 16microSec
	; OCR1A = 6249
	; Actual Time = (6249 + 1) * 256/ 16000000 = 0.1 second which is better with the 0.1 second 
    ldi r16, 0b00001100           ; CS12:10 = 100 = Prescaler 256
    sts TCCR1B, r16
    
    ; CHANGED: OCR1A from 7812 to 6249
    ; OCR1A = (0.1 * 16,000,000) / 256 - 1 = 6249 = 0x1869
    ldi r16, high(6249)
    sts OCR1AH, r16
    ldi r16, low(6249)
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
; Triggered every 0.1 seconds (10 Hz)
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
