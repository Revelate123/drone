; ============================================================================
; keypad_simple.inc - Fixed version
; ============================================================================

jmp end_keypad_simple

; ============================================================================
; PROCESS_KEY - FIXED VERSION
; ============================================================================
process_key:
    push r17
    
    ; Handle pause first (Key 0 hold = pause, release = resume)
    cpi r16, '0'
    breq key_pause_hold
    
    ; No key or other key pressed - resume if paused
    lds r17, drone_state
    cpi r17, 'P'
    brne check_other_keys
    
    ; Was paused, now resume
    ldi r17, 'F'
    sts drone_state, r17
    
check_other_keys:
    cpi r16, 0xFF           ; No key pressed?
    breq process_key_end
    
    ; Key 1 = Altitude UP
    cpi r16, '1'
    breq key_alt_up
    
    ; Key 4 = Altitude DOWN
    cpi r16, '4'
    breq key_alt_down
    
    ; Key 2 = Speed UP
    cpi r16, '2'
    breq key_speed_up
    
    ; Key 5 = Speed DOWN
    cpi r16, '5'
    breq key_speed_down
    
    rjmp process_key_end

key_pause_hold:
    ; Holding Key 0 = Pause
    ldi r17, 'P'
    sts drone_state, r17
    rjmp process_key_end

key_alt_up:
    lds r16, drone_altitude
    cpi r16, 100
    brsh process_key_end
    inc r16
    sts drone_altitude, r16
    rjmp process_key_end

key_alt_down:
    lds r16, drone_altitude
    cpi r16, 0
    breq process_key_end
    dec r16
    sts drone_altitude, r16
    rjmp process_key_end

key_speed_up:
    lds r16, drone_speed
    cpi r16, 20
    brsh process_key_end
    inc r16
    sts drone_speed, r16
    rjmp process_key_end

key_speed_down:
    lds r16, drone_speed
    cpi r16, 1              ; Don't go below 1
    breq process_key_end
    dec r16
    sts drone_speed, r16
    rjmp process_key_end

process_key_end:
    pop r17
    ret

; ============================================================================
; INIT_KEYPAD_SIMPLE
; ============================================================================
init_keypad_simple:
    push r16
    
    ldi r16, 0xF0
    sts DDRL, r16
    
    ldi r16, 0x0F
    sts PORTL, r16
    
    pop r16
    ret

; ============================================================================
; SCAN_KEYPAD_SIMPLE
; Returns: r16 = ASCII character or 0xFF if no key
; ============================================================================
scan_keypad_simple:
    push r17
    push r18
    push r19
    push r20
    push r21
    
    ldi r19, 0xEF
    clr r17

scan_colloop:
    cpi r17, 4
    breq scan_no_key
    
    sts PORTL, r19
    
    ldi r20, 0xFF
scan_delay:
    dec r20
    brne scan_delay

    lds r20, PINL
    andi r20, 0x0F
    cpi r20, 0x0F
    breq scan_nextcol
    
    ldi r18, 0x01
    clr r16
    
scan_rowloop:
    cpi r16, 4
    breq scan_nextcol
    mov r21, r20
    and r21, r18
    breq scan_convert
    inc r16
    lsl r18
    jmp scan_rowloop

scan_nextcol:
    lsl r19
    inc r17
    jmp scan_colloop

scan_convert:
    cpi r17, 3
    breq scan_letters
    cpi r16, 3
    breq scan_symbols

    mov r20, r16
    lsl r20
    add r20, r16
    add r20, r17
    subi r20, -'1'
    mov r16, r20
    jmp scan_end

scan_letters:
    ldi r20, 'A'
    add r20, r16
    mov r16, r20
    jmp scan_end

scan_symbols:
    cpi r17, 0
    breq scan_star
    cpi r17, 1
    breq scan_zero
    ldi r16, '#'
    jmp scan_end
    
scan_star:
    ldi r16, '*'
    jmp scan_end
    
scan_zero:
    ldi r16, '0'
    jmp scan_end

scan_no_key:
    ldi r16, 0xFF

scan_end:
    pop r21
    pop r20
    pop r19
    pop r18
    pop r17
    ret

end_keypad_simple: