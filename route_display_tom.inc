;
; Test.asm
;
; Created: 10/10/2025 11:56:26 pm
; Author : thoma
;


rjmp end_route_display_tom

.equ button1 = 1;
.equ button2 = 0;
.equ loop_count = 0xFFFF
.equ inner_loop_count = 0x0A

.def state = r20
.def inner = r23
.def iH = r25
.def iL = r24
.def countH = r17
.def countL = r16

; This is the lecture code modified with an inner loop and nop replaced by
; check_button_pressed
.macro timed_poll
	ldi countL, low(loop_count)
	ldi countH, high(loop_count)
	clr iH
	clr iL
loop: cp iL, countL
	cpc iH, countH
	brsh done
	adiw iH:iL, 1
	ldi inner, inner_loop_count
	loop_inner:
		dec inner
		check_button_pressed
		brne loop_inner
	rjmp loop
done:
.endmacro

; This is the lecture code modified with an inner loop
.macro wait_for_bounce
	ldi countL, low(loop_count)
	ldi countH, high(loop_count)
	clr iH
	clr iL
loop: cp iL, countL
	cpc iH, countH
	brsh done
	adiw iH:iL, 1
	ldi inner, inner_loop_count
	loop_inner:
		dec inner
		nop
		brne loop_inner
	rjmp loop
done:
.endmacro


.macro check_button_pressed
	sbic PIND, button2		; skip next instruction if button pressed
	rjmp exit
button_pressed:	
	wait_for_bounce			; prevents accidental multiple presses due to bouncing
	jmp done_waiting_for_pb1
exit:
.endmacro


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
cur_route_display: .byte 15*9
.cseg
end_route_display_tom: