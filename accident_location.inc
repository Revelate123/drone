
jmp accident_location_end

check_accident_visible:
check_accident_visible_prologue:
	push r16
    push r17
check_accident_visible_body:
	; Check if 

check_accident_visible_epligoue:
    ; Compare X coordinates
    lds r16, accident_x
    lds r17, drone_current_x
    cp r16, r17
    brne not_found_al ; If X doesn't match, exit
    
    ; Compare Y coordinates
    lds r16, accident_y
    lds r17, drone_current_y
    cp r16, r17
    brne not_found_al ; If Y doesn't match, exit
    
    ; Both coordinates match - accident found
    ldi r16, 1
    sts main_crash_site_found, r16
    ldi r16, 'A'
	sts drone_state, r16 ; Set search result flag
    
    
not_found_al:
	pop r17
    pop r16
    ret


display_crash_location:
    push r16
	push r17
    push r20
    push r21
    
    ; Move cursor to start of line 2 (bottom row)
    do_lcd_command 0b11000000  ; 0xC0 = line 2, position 0
    
	 ; Clear the entire bottom row (16 spaces)
    ldi r17, 16
clear_bottom_row:
    do_lcd_data ' '
    dec r17
    brne clear_bottom_row

    
    ; Move cursor back to start of line 2
    do_lcd_command 0b11000000  ; 0xC0 = line 2, position 0

    ; Display opening parenthesis
    do_lcd_data '('
    
    ; Load and display crash_x
    lds r20, crash_x
    
    ; Check for special case '*'
    cpi r20, '*'
    breq crash_x_asterisk
    
    ; Regular number display
    cpi r20, 10
    brsh crash_x_two_digits
    do_lcd_data ' '      
    subi r20, -'0'           
    do_lcd_data_from_reg r20
    rjmp crash_x_done
    
crash_x_two_digits:
    ldi r16, '1'
    do_lcd_data_from_reg r16
    subi r20, 10
    subi r20, -'0'
    do_lcd_data_from_reg r20
    rjmp crash_x_done
    
crash_x_asterisk:
    do_lcd_data '*'
    
crash_x_done:
    ; Display comma
    do_lcd_data ','
    
    ; Load and display crash_y
    lds r21, crash_y
    
    ; Check for special case '*'
    cpi r21, '*'
    breq crash_y_asterisk
    
    ; Regular number display
    cpi r21, 10
    brsh crash_y_two_digits
    do_lcd_data ' '     
    subi r21, -'0'        
    do_lcd_data_from_reg r21
    rjmp crash_y_done
    
crash_y_two_digits:
    ldi r16, '1'
    do_lcd_data_from_reg r16
    subi r21, 10
    subi r21, -'0'
    do_lcd_data_from_reg r21
    rjmp crash_y_done
    
crash_y_asterisk:
    do_lcd_data '*'
    
crash_y_done:
    ; Display closing parenthesis
    do_lcd_data ')'
    
    pop r21
    pop r20
	pop r17
    pop r16
    ret

accident_location_end:
	