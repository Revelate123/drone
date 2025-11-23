
; keypad_simple.inc 


jmp end_keypad_simple


; PROCESS_KEY_NO_PAUSE - Altitude and Speed only (NOT pause)

process_key_no_pause:
    push r17
    
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

key_alt_up:
    lds r16, drone_altitude
    cpi r16, 15
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
    cpi r16, 1
    breq process_key_end
    dec r16
    sts drone_speed, r16
    rjmp process_key_end

process_key_end:
	
    pop r17
    ret


; INIT_KEYPAD
init_keypad_simple:
    push r16
    
    ldi r16, 0xF0
    sts DDRL, r16
    
    ldi r16, 0x0F
    sts PORTL, r16
    
    pop r16
    ret


; SCAN_KEYPAD_SIMPLE
; Returns: r16 = ASCII character or 0xFF if no key
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

wait_key_release:
	call delay_
wait_release_loop:
	lds r21, PINL
    andi r21, 0x0F
    cpi r21, 0x0F
    brne wait_release_loop
	call delay_
	ret

delay_:
	push r22
	push r23
	push r24
	ldi r23, 255
	ldi r24, 6
delay_outer:
	ldi r22, 255
delay_inner:
	dec r22
	brne delay_inner
	dec r23
	brne delay_outer
	dec r24
	brne delay_outer
	pop r24
	pop r23
	pop r22
	ret

printing_initial_display_1:
msg_location: .db "map(M*M); M=",0
	input_msg_to_display msg_location, 0

	push ZL
	push ZH
	push r16
	push r17
		ldi ZL, low(map_size<<1)
		ldi ZH, high(map_size<<1)
		lpm r16, Z
		cpi r16, 10
		brlo print_map_size_lower_than_10
		subi r16, 10
		mov r17, r16
		do_lcd_data '1'
		mov r16, r17
	print_map_size_lower_than_10:
		subi r16, -'0'
		do_lcd_data_from_reg r16
	pop r17
	pop r16
	pop ZH
	pop ZL
printing_initial_display_1_epilogue:
	do_lcd_command 0b11000000
	msg_location_1: .db "loc:",0
	input_msg_to_display msg_location_1, 0
	ret

get_crash_xy:
; using keypad for user input
; showing keypad presses into lcd and saves it to memory
; saves it in r17 before putting it to memory
	rcall printing_initial_display_1
get_crash_xy_prologue:
	push r16						; used for taking value of scan_keypad_simple
	push r17						; saving the value to memory
	push r18						; temp register
	clr r17
	rcall init_keypad_simple
get_crash_xy_loop:
get_x1:
	rcall scan_keypad_simple
	cpi r16, 0xFF					; scanning for 0xFF which means no key is pressd
	breq get_x1
	cpi r16, '*'					; scan for * meaning that no accident
	brne continue_check_digits_for_crash
	rjmp no_accident

continue_check_digits_for_crash:
	cpi r16, '0'					; scanning for inputs of '0'-'9' only
	brlo get_x1						; should return and get other value
	cpi r16, '9'+1
	brsh get_x1

	do_lcd_data_from_reg r16		; show value and save to memory
	subi r16, '0'
	mov r17, r16

	rcall wait_key_release

get_x2_or_hash:
	rcall scan_keypad_simple
	cpi r16, 0xFF					; scanning for 0xFF which means no key is pressd
	breq get_x2_or_hash
	cpi r16, '#'
	breq get_y1_
	cpi r16, '0'					; scanning for inputs of '0'-'9' only
	brlo get_x2_or_hash				; should return and get other value
	cpi r16, '9'+1
	brsh get_x2_or_hash

	do_lcd_data_from_reg r16		; show value and save to memory
	subi r16, '0'					; mul 1st digit by 10 + 2nd digit
	lsl r17
	mov r18, r17
	lsl r17
	lsl r17
	add r17, r18
	add r17, r16

	rcall wait_key_release

get_y1_:
	sts crash_x, r17
	do_lcd_data ','
get_y1:
	rcall scan_keypad_simple
	cpi r16, 0xFF					; scanning for 0xFF which means no key is pressd
	breq get_y1
	cpi r16, '0'					; scanning for inputs of '0'-'9' only
	brlo get_y1						; should return and get other value
	cpi r16, '9'+1
	brsh get_y1

	do_lcd_data_from_reg r16		; TODO: save r16 value here!!!
	subi r16, '0'
	mov r17, r16

	rcall wait_key_release

get_y2_or_hash:
	rcall scan_keypad_simple
	cpi r16, 0xFF					; scanning for 0xFF which means no key is pressd
	breq get_y2_or_hash
	cpi r16, '#'
	breq get_crash_xy_epilogue
	cpi r16, '0'					; scanning for inputs of '0'-'9' only
	brlo get_y2_or_hash				; should return and get other value
	cpi r16, '9'+1
	brsh get_y2_or_hash

	do_lcd_data_from_reg r16		; show value and save to memory
	subi r16, '0'					; mul 1st digit by 10 + 2nd digit
	lsl r17
	mov r18, r17
	lsl r17
	lsl r17
	add r17, r18
	add r17, r16

	rcall wait_key_release
	rjmp get_crash_xy_epilogue

no_accident:
	do_lcd_data '*'
	do_lcd_data ','
	do_lcd_data '*'
	ldi r17, '*'
	sts crash_x, r17

get_crash_xy_epilogue:
	sts crash_y, r17
	pop r18
	pop r17
	pop r16
	ret

get_visibility_value:
; using keypad for user input
; showing keypad presses into lcd and saves it to memory
; will return after inputting 2 digits
; will max at 26 for user input
; if user inputs "3" as first digit, it will directly save it
; if user inputs "2" and ("6" or "7" or "8" or "9") will automatically show and save "6"

msg_visibility_1: .db "Input visibility",0
msg_visibility_2: .db "(0-26):",0
	input_msg_to_display msg_visibility_1, 0
    do_lcd_command 0b11000000
	input_msg_to_display msg_visibility_2, 0

get_vis_prologue:
	push r16						; used for taking value of scan_keypad_simple
	push r17						; saving the value to memory
	push r18						; temp
	rcall init_keypad_simple
get_vis_loop:
get_dig1:
	rcall scan_keypad_simple
	cpi r16, 0xFF					; scanning for 0xFF which means no key is pressd
	breq get_dig1
	cpi r16, '0'					; scanning for inputs of '0'-'9' only
	brlo get_dig1					; should return and get other value
	cpi r16, '9'+1
	brsh get_dig1
	cpi r16, '2'+1
	brlo found_2_or_below
	; if it is > 2, automatically cap and finalise
	do_lcd_data_from_reg r16		; show value and save to memory
	subi r16, '0'
	mov r17, r16
	rcall wait_key_release
	rjmp get_vis_epilogue

found_2_or_below:
	do_lcd_data_from_reg r16		; show value and save to memory
	subi r16, '0'
	mov r17, r16
	rcall wait_key_release
	cpi r16, 2
	breq get_dig2_or_hash_after_2
get_dig2_or_hash_after_1:
	rcall scan_keypad_simple
	cpi r16, 0xFF					; scanning for 0xFF which means no key is pressd
	breq get_dig2_or_hash_after_1
	cpi r16, '#'					; found # then move on to saving the value
	breq get_vis_epilogue
	cpi r16, '0'					; scanning for inputs of '0'-'9' only
	brlo get_dig2_or_hash_after_1	; should return and get other value
	cpi r16, '9'+1
	brsh get_dig2_or_hash_after_1
	rjmp save_dig2
get_dig2_or_hash_after_2:
	rcall scan_keypad_simple
	cpi r16, 0xFF					; scanning for 0xFF which means no key is pressd
	breq get_dig2_or_hash_after_1
	cpi r16, '#'					; found # then move on to saving the value
	breq get_vis_epilogue
	cpi r16, '0'					; scanning for inputs of '0'-'9' only
	brlo get_dig2_or_hash_after_1	; should return and get other value
	cpi r16, '9'+1
	brsh get_dig2_or_hash_after_1
	cpi r16, '6'
	brsh save_as_26
	rjmp save_dig2
save_as_26:
	ldi r16, '6'
save_dig2:
	do_lcd_data_from_reg r16		; show value and save to memory
	subi r16, '0'					; mul 1st digit by 10 + 2nd digit
	lsl r17
	mov r18, r17
	lsl r17
	lsl r17
	add r17, r18
	add r17, r16

	rcall wait_key_release
get_vis_epilogue:
	sts vis_val, r17
	pop r18
	pop r17
	pop r16
	ret

end_keypad_simple:
