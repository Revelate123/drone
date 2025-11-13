; ============================================================================
; led.inc
; ============================================================================
; LED control for status indication
; ============================================================================

jmp end_led

; ============================================================================
; INIT_LED
; Configures PORTC for LED bar output
; ============================================================================
init_led:
    push r16
    
    ldi r16, 0xFF
    out DDRC, r16
    
    ldi r16, 0xFF
    out PORTC, r16
    
    pop r16
    ret

; ============================================================================
; INIT_LED_FLASH
; Initialize LED flash state
; ============================================================================
init_led_flash:
    push r16
    
    ldi r16, 0
    sts led_flash_counter, r16
    
    ldi r16, 1
    sts led_state, r16
    
    pop r16
    ret

; ============================================================================
; FLASH_LED_CONTINUOUS
; Toggles LED for continuous flashing when crashed
; ============================================================================
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

end_led: