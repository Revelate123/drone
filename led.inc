
; led.inc


jmp end_led

; INIT_LED
; Configures PORTC for LED bar output
init_led:
    push r16
    
    ldi r16, 0xFF
    out DDRC, r16
    
    ldi r16, 0x00      ;  Start with LEDs OFF
    out PORTC, r16
    
    pop r16
    ret
	
; INIT_LED_FLASH
; Initialize LED flash state
init_led_flash:
    push r16
    
    ldi r16, 0
    sts led_flash_counter, r16
    
    ldi r16, 1
    sts led_state, r16
    
    pop r16
    ret


; FLASH_LED_CONTINUOUS
; Toggles LED for continuous flashing when crashed
flash_led_continuous:
    push r16
    push r17
    
    lds r16, led_flash_counter
    inc r16
    sts led_flash_counter, r16
    
    cpi r16, 5
    brlo flash_led_done
    
    ldi r16, 0
    sts led_flash_counter, r16
    
    lds r16, led_state
    cpi r16, 0
    breq flash_led_turn_on
    
    ldi r16, 0x00
    out PORTC, r16
    ldi r16, 0
    sts led_state, r16
    rjmp flash_led_done
    
flash_led_turn_on:
    ldi r16, 0xFF
    out PORTC, r16
    ldi r16, 1
    sts led_state, r16
    
flash_led_done:
    pop r17
    pop r16
    ret

; SHOW_LED_PROGRESS
; Shows progress by lighting up more LEDs
; Input: r16 = number of LEDs to light (1-8)
; Used for testing purpose
show_led_progress:
    push r17
    push r18
    
    clr r17              
    clr r18              
    
    cpi r16, 0           
    breq show_zero_leds
    
    ; Cap at 8 LEDs maximum
    cpi r16, 9
    brlo build_pattern
    ldi r16, 8           ; If more than 8, set to 8
    
build_pattern:
    cp r18, r16          
    breq pattern_done
    
    sec                  
    rol r17              
    inc r18
    rjmp build_pattern
    
pattern_done:
    mov r16, r17
    out PORTC, r16       
    rjmp show_done
    
show_zero_leds:
    ldi r16, 0x00
    out PORTC, r16
    
show_done:
    pop r18
    pop r17
    ret
end_led:
