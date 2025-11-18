;
; Test.asm
;
; Created: 10/10/2025 11:56:26 pm
; Author : thoma
;


rjmp end_route_display_tom

.def char_count = r17
.def cur_x = r20
.def cur_y = r21
.def cur_z = r22
store_display_route:
store_display_route_prologue:
push XL
push XH
push r16
push r18
push r19
push r20
push r21
push r22
store_display_route_body:

	ldi XL, low(route_size)
	ldi XH, high(route_size)
	ldi YL, low(cur_route_display)
	ldi YH, high(cur_route_display)

	clr char_count
	ld r24, X+
	sdr_loop:
		ld cur_x, X+
		ld cur_y, X+
		ld cur_z, X+
		mov r16, cur_x
		rcall number_to_ascii
		ldi r16, ','
		st Y+, r16
		inc char_count
		mov r16, cur_y
		rcall number_to_ascii
		ldi r16, ','
		st Y+, r16
		inc char_count
		mov r16, cur_z
		rcall number_to_ascii 
		ldi r16, '/'
		st Y+, r16
		inc char_count

		dec r24
		brne sdr_loop


store_display_route_epilogue:
pop r22
pop r21
pop r20
pop r19
pop r18
pop r16
pop XH
pop XL
ret

store_display_route2:
store_display_route2_prologue:
push XL
push XH
push r16
push r18
push r19
push r20
push r21
push r22
store_display_route2_body:

	ldi XL, low(cur_route_size)
	ldi XH, high(cur_route_size)
	ldi YL, low(route_between_points)
	ldi YH, high(route_between_points)

	
	ld r24, X+
	sdr2_loop:
		ldi char_count, 3
		ld cur_x, X+
		inc char_count
		cpi cur_x, 10
		brsh two_digit_x
		rjmp check_y_digits
two_digit_x:
		inc char_count
check_y_digits:
		ld cur_y, X+
		inc char_count
		cpi cur_y, 10
		brsh two_digit_y
		rjmp check_z_digits
two_digit_y:
		inc char_count
check_z_digits:
		call get_value_at_coords ; loads to cur_z
		inc char_count
		cpi cur_z, 10
		brsh two_digit_z
		rjmp store_digits
two_digit_z:
		inc char_count
store_digits:
		; First store number of chars 
		st Y+, char_count
		mov r16, cur_x
		rcall number_to_ascii
		ldi r16, ','
		st Y+, r16
		inc char_count
		mov r16, cur_y
		rcall number_to_ascii
		ldi r16, ','
		st Y+, r16
		inc char_count
		mov r16, cur_z
		rcall number_to_ascii 
		ldi r16, '/'
		st Y+, r16
		inc char_count

		dec r24
		brne sdr2_loop


store_display_route2_epilogue:
pop r22
pop r21
pop r20
pop r19
pop r18
pop r16
pop XH
pop XL
ret

.def pointer = r18
.def counter = r19
display_route_points_tom:
display_route_points_prologue:
push r16
push r18
push r20
push r21
display_route_points_body:
call store_display_route ; stores char_count in r17
// We want to fill 16 chars of the lcd and then push and pop from the ends
	ldi YL, low(cur_route_display)
	ldi YH, high(cur_route_display)
//repeatedly fill with chars 16 times.
do_lcd_command 0b00001100
do_lcd_command 0b00110100

ldi r25, 16
clr pointer
clr counter
clr r21
drpt_loop:
	//if pointer == char_count: repeat
	ld pointer, Y+
	inc counter
	do_lcd_data_from_reg pointer

	cp counter, char_count
	breq reset_counter1
	rjmp continue1
reset_counter1:
	clr counter
	sub YL, char_count
	sbc YH, counter
continue1:
	dec r25
	brne drpt_loop

display_route_points_epilogue:
	pop r21
	pop r20
	pop r18
	pop r16
	ret

update_display_route:
update_display_route_prologue:
push r16
push r18
push r20
push r21
update_display_route_body:
// Now the LCD should be filled with 16 characters?
do_lcd_command 0b00000111 ; change entry mode to shift left

drpt_loop2:
	//if pointer == char_count: repeat
	ld pointer, Y+
	do_lcd_data_from_reg pointer
	inc counter
	cp counter, char_count
	breq reset_counter2
	rjmp continue2
reset_counter2:
	clr counter
	sub YL, char_count
	sbc YH, counter
continue2:
;call sleep_500ms
update_display_route_epilogue:
	pop r21
	pop r20
	pop r18
	pop r16
	ret













display_route_points_tom2:
display_route_points2_prologue:
push r16
push r18
push r20
push r21
display_route_points2_body:
call store_display_route2 ; stores char_count in r17
// We want to fill 16 chars of the lcd and then push and pop from the ends
reset_screen
	ldi YL, low(route_between_points)
	ldi YH, high(route_between_points)

ld counter, Y+
sts chars_in_cur_pos, counter

ldi r25, 16
clr pointer
clr r21
drpt2_loop:
	
	ld pointer, Y+
	
	do_lcd_data_from_reg pointer
	dec counter
	breq skip_next
	rjmp continue3
skip_next:
	ld counter, Y+
continue3:
	dec r25
	brne drpt2_loop

display_route_points2_epilogue:
	pop r21
	pop r20
	pop r18
	pop r16
	ret






update_display_route1:
update_display_route_prologue1:
push r16
push r18
push r20
push r21
update_display_route_body2:
reset_screen
	ldi YL, low(route_between_points) ; Loads from the start, which is an error
	ldi YH, high(route_between_points) ; Probably should load the current position in memory and then step through by cur_chars. AFter that save the new pos.
; ---------This block is for advancing to the next point in the path----
adiw Y ,1
lds counter, chars_in_cur_pos
advance_disp:
	adiw Y, 1
	dec counter
	brne advance_disp
; ----------------------------------------------------------------------
; Probably save the YL and YH in memory now.
ld counter, Y+
sts chars_in_cur_pos, counter

ldi r25, 16 ; Also need to consider if we are at the end of the route, then we should probably just print nothing/exit
clr pointer
clr r21
drpt3_loop:
	
	ld pointer, Y+
	
	do_lcd_data_from_reg pointer
	dec counter
	breq skip_next2
	rjmp continue4
skip_next2:
	ld counter, Y+ ; Maybe if counter is a value e.g. 0xff we break. THis would require adding one extra store at the end of store_display_route2
continue4:
	dec r25
	brne drpt3_loop
update_display_route_epilogue2:
	pop r21
	pop r20
	pop r18
	pop r16
	ret











.def input_value=R16
.def sign=R17					
.def hundreds=R18				
.def tens=R19
.def ones=R20

number_to_ascii:
number_to_ascii_prologue:
push r16
push r18
push r19
push r20
number_to_ascii_body:
ldi hundreds, 0x30				;set hundreds to 0
ldi tens, 0x30					;set tens to 0
ldi ones, 0x30					;set ones to 0


tens_block:
	cpi input_value, 10			; loop to execute while input_value >= 10
	brge loop_tens
	rjmp ones_block
	loop_tens:					; increment tens and decrement input_value by 10
		inc tens
		subi input_value, 10
		cpi input_value, 10
		brge loop_tens			; repeat while input_value >= 10

ones_block:
	cpi input_value, 1			; loop to execute while input_value > 0
	brge loop_ones
	rjmp final
	loop_ones:					; increment ones and decrement input_value by 1
		inc ones
		subi input_value, 1
		cpi input_value, 1
		brge loop_ones			; repeat while input_value > 0

		final:
cpi tens, '0'
breq store_ones
st Y+, tens
inc r17
store_ones:
st Y+, ones
inc r17
number_to_ascii_epilogue:
pop r20
pop r19
pop r18
pop r16
ret

.dseg
cur_route_display: .byte 225*9

chars_in_cur_pos: .byte 1
route_between_points: .byte 15*9
.cseg
end_route_display_tom:
