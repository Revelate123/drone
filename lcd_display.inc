;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; CODE 2 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Test code from Hui Wu
; Board settings: 1. Connect LCD data pins D0-D7 to PORTF0-7.
; 2. Connect the four LCD control pins BE-RS to PORTA4-7.

;----------------------------------------------------------------------------------------------------
; This example code from the lecture has been modified to be used as a macro more easily.			|
;----------------------------------------------------------------------------------------------------

.include "m2560def.inc"

;------------------------------------------------------------------------------
;Jumps to the end of the file.	
;This prevents code from running when the file is used via .include
rjmp end_of_file
;------------------------------------------------------------------------------


.macro do_lcd_command
	ldi r16, @0
	call lcd_command
	call lcd_wait
.endmacro


.macro do_lcd_data_from_reg
	mov r16, @0
	call lcd_data
	call lcd_wait
.endmacro

.macro do_lcd_data
	ldi r16, @0
	call lcd_data
	call lcd_wait
.endmacro

;----------------------------------------------------------------------------------------------------
; RESET was changed to the macro reset_screen and the default data displayed was deleted.
;----------------------------------------------------------------------------------------------------
.macro reset_screen

	ser r16
	out DDRF, r16
	out DDRA, r16
	clr r16
	out PORTF, r16
	out PORTA, r16

	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink
.endmacro


.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

.macro lcd_set
	sbi PORTA, @0
.endmacro
.macro lcd_clr
	cbi PORTA, @0
.endmacro

.macro lcd_shift_left
	do_lcd_command 0b00011000
.endmacro
.macro lcd_back_to_home
	do_lcd_command 0b00000010
.endmacro

.macro input_msg_to_display
	; @0 is the message from pmem
	; assuming it fits 16 chars
	; @1=0: keep cursor at the end, @1=1: move cursor back to home
	; uses ZL, ZH, r17, r18, r19 pls do stack init for this
	; clear counter (r17)
	; temp = r18
	; delay counter (r19)
	; keeping cursor option (r20)
	push r17
	push r18
	push r20
	push ZL
	push ZH
	ldi ZL, low(@0<<1)
	ldi ZH, high(@0<<1)
	ldi r20, @1
	rcall print_multiple
	pop ZH
	pop ZL
	pop r20
	pop r18
	pop r17

.endmacro

print_multiple:
	clr r17
print_loop:
	lpm r18, Z+
	tst r18
	breq print_done
	do_lcd_data_from_reg r18
	inc r17						; increase counter
	rjmp print_loop

print_done:
/*	; if r17 counter is more than 16 (visible LCD) -> scroll
	mov r18, r17
	cpi r18, 17
	brlo fin_scrolling
lcd_overflow_check:
	cpi r18, 17
	brlo fin_scrolling
	lcd_shift_left
	rcall sleep_500ms
	dec r18
	rjmp lcd_overflow_check
*/
fin_scrolling:
	tst r20
	breq skip_back_to_home
	lcd_back_to_home
skip_back_to_home:
	;rcall sleep_500ms
	ret

; ===========================================================
; displaying route and updating route for search
; ============================================================

display_route_points:
; save printed values in ascii to dseg: route_printed
; prints the first 16 chars of path_printed
display_route_prologue:
	reset_screen
	push XL
	push XH
	push r16
	push r17	; this is route_size
	push r18	; this is for the content
	push r19	; this is for char_counter
	push r20	; this is for x, y, z stages
				; found x. r20 = 0 || found y. r20 = 1 || found z. r20 = 2
	push r21	; this is for temp
	push YL
	push YH
	ldi XL, low(route_size)
	ldi XH, high(route_size)
	ldi YL, low(route_printed)
	ldi YH, high(route_printed)
	; we want to read the next route_size * 3
	sts cur_RtDisp_addr, YL
	sts cur_RtDisp_addr+1, YH
	ldi r16, 3	; 3: this is for x, y, z
	ld r17, X+
	mul r17, r16
	mov r17, r0 ; now r17 contains how much next byte to read
	clr r20
	clr r1
	clr r16
	clr r19

display_route_loop:
	ld r18, X+					; value saved in r18; max 15
	dec r17						; route value printed -1

	; print here, max value is 15

	cpi r18, 10
	brlo lower_than_10
	ldi r21, '1'				; fill temp with value '1'
	st Y+, r21					; store to Y/route-printed for printed value
	subi r18, 10				; -10 for the next digit to show up
	inc r19						; inc counter for char display

lower_than_10:
	subi r18, -'0'				; digit to ascii
	st Y+, r18					; store to Y/route-printed for printed value
	inc r19						; inc counter for char display

	cpi r20, 2					; stage check
	brsh found_xyz

	ldi r21, ','
	st Y+, r21					; store to Y/route-printed for printed value
	inc r19						; inc counter for char display
	inc r20						; inc counter for stage; find x->y->z
	rjmp display_route_loop		; find next value
	
found_xyz:						; found (x,y,z) complete pair
	ldi r21, '/'
	st Y+, r21					; store to Y/route-printed for printed value
	inc r19						; inc counter for char display
	clr r20						; clear stage
	cpi r17, 0					; compare with 0, end of route?
	brne display_route_loop		; find next value set
	ldi r21, 0
	st Y+, r21					; store 0 to mark end of route?


print_route_first_16:
	ldi YL, low(route_printed)
	ldi YH, high(route_printed)
	ldi r21, 16
print_route_loop:
	ld r18, Y+					; load char pointed in Y to r18
	do_lcd_data_from_reg r18
	dec r21
	cpi r21, 0
	brne print_route_loop
display_route_epilogue:
	pop YH
	pop YL
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	pop XH
	pop XL
	ret

update_route:
	; display next point in route by scrolling from current point -> next
			; reset_screen
			; print from current_point + 25 chars (16+9)
			; scrolls to the next '/'
	; call this function to traverse to next point in display
	; TODO: should set cur_RtDisp_addr => route_printed before this
update_route_prologue:
	reset_screen
	push XL
	push XH
	push r17	
	push r16
	lds XL, cur_RtDisp_addr
	lds XH, cur_RtDisp_addr+1
	ldi r17, 25							; counter 25 chars
prints_route_25_chars_after_address:
	ld r16, X+						; load char pointed in X to r16
	do_lcd_data_from_reg r16
	dec r17
	cpi r16, 0						; finds eof (end of route) move to current address and start scrolling
	breq scroll_route_init
	cpi r17, 0						
	brne prints_route_25_chars_after_address

scroll_route_init:
	lds XL, cur_RtDisp_addr
	lds XH, cur_RtDisp_addr+1
update_route_loop:
	ld r16, X+
	cpi r16, '/'
	breq found_slash
	lcd_shift_left
	rcall sleep_100ms
	rjmp update_route_loop
found_slash:
	lcd_shift_left
	rcall sleep_100ms
	sts cur_RtDisp_addr, XL
	sts cur_RtDisp_addr+1, XH
	call update_speed
	call update_height
	call update_status
update_route_epilogue:
	pop r16
	pop r17
	pop XH
	pop XL
	ret


; ===========================================================
; displaying path and updating path in between two points
; ============================================================

display_path_between_two_points:
; prints path to lcd
; save printed path values in ascii to dseg: path_printed
; will not scroll in the first initialisation of display
display_path_2_prologue:
	reset_screen
	push XL
	push XH
	push YL
	push YH
	push r16	; can be used for temp
	push r17	; this is path_size showing how many bytes left to read
	push r18	; temp
	push r19	; this is for display_char_counter
	push r20	; this is for the content of the byte (x)
	push r21	; this is for the content of the byte (y)
	push r22	; this is for the content of the byte (z)

	ldi XL, low(cur_route_size)
	ldi XH, high(cur_route_size)
	ldi YL, low(path_printed)
	ldi YH, high(path_printed)
	; we want to read the next route_size * 3
	ldi r16, 2	; 2: this is for x, y
	ld r17, X+

	mul r17, r16
	mov r17, r0 ; now r17 contains how much byte to read
	clr r1
	clr r19

display_path_2_loop:
path_get_x:
	ld r20, X+					; value x saved in r20; max 15
	dec r17						; route value printed -1
	cpi r20, 10
	brlo path_x_lo_10

	ldi r16, '1'
	do_lcd_data_from_reg r16
	st Y+, r16					; store to Y/path-printed for printed value
	subi r20, 10
	inc r19						; inc char displayed counter

path_x_lo_10:
	subi r20, -'0'				; digit to ascii
	do_lcd_data_from_reg r20	; print 1s, printing from r20
	st Y+, r20					; store to Y/path-printed for printed value
	inc r19						; inc counter for char display

	ldi r16, ','
	do_lcd_data_from_reg r16
	st Y+, r16					; store to Y/path-printed for printed value
	inc r19

path_get_y:
	ld r21, X+					; saving y value now in r21
	dec r17						; route value printed -1
	cpi r21, 10					; check if more than 10
	brlo path_y_lo_10

	ldi r16, '1'				; yes, print 1
	do_lcd_data_from_reg r16
	st Y+, r16					; store to Y/path-printed for printed value
	subi r21, 10				; take second digit
	inc r19						; inc char displayed counter

path_y_lo_10:
	subi r21, -'0'				; digit to ascii
	do_lcd_data_from_reg r21	; print 1s, printing from r21
	st Y+, r21					; store to Y/path-printed for printed value
	inc r19						; inc counter for char display

	ldi r16, ','
	do_lcd_data_from_reg r16
	st Y+, r16					; store to Y/path-printed for printed value
	inc r19

path_get_z:
	subi r20, '0'				; returning value of x to non-ascii
	subi r21, '0'				; returning value of y to non-ascii
	rcall get_value_at_coords	; z value saved at r22
	cpi r22, 10					; check if z is more than 10
	brlo path_z_lo_10
	
	ldi r16, '1'
	do_lcd_data_from_reg r16
	st Y+, r16
	subi r22, 10
	inc r19

path_z_lo_10:
	subi r22, -'0'
	do_lcd_data_from_reg r22	; print 1s, printing from r22
	st Y+, r22					; store to Y/path-printed for printed value
	inc r19						; inc counter for char display

found_xyz_:						; BRANCH found (x,y,z) complete pair
	ldi r16, '/'
	do_lcd_data_from_reg r16
	st Y+, r16					; store to Y/path-printed for printed value
	inc r19

	cpi r17, 0					; compare with 0, end of route?
	breq dont_loop_back
	rjmp display_path_2_loop	; BRANCH find next value set
dont_loop_back:
	rcall sleep_500ms
lcd_overflow_check__:			; BRANCH
	cpi r19, 17					; compare length of data to max display 0-16 chars
	brlo display_path_2_epilogue
	lcd_shift_left
	rcall sleep_500ms
	dec r19
	rjmp lcd_overflow_check__	; BRANCH

display_path_2_epilogue:		; BRANCH
	rcall sleep_500ms
	lcd_back_to_home
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	pop YH
	pop YL
	pop XH
	pop XL
	ret

update_path:
	; display next point in path between 2 pts by scrolling
	; call this function to traverse to next point in display
update_path_prologue:
update_path_loop:
update_path_epilogue:
	ret



update_status:
	; r21 includes the status
	; r21 = 0 Fly
	; r21 = 1 Hover/Pause
	; r21 = 2 Crash
update_status_prologue:
	push r21
	push r16
update_status_body:
	do_lcd_command 0b11000001
	lds r21, drone_state
	do_lcd_data_from_reg r21
	rjmp update_status_epilogue
update_status_epilogue:
	pop r16
	pop r21
	ret

update_speed:
	; r21 had the speed value in dm/s
	; max is 29 m/s
update_speed_prologue:
push r21
push r16
update_speed_body:
	do_lcd_command 0b11000110
	lds r21, drone_speed
	cpi r21, 20
	brsh print_20s
	cpi r21, 10
	brsh print_10s
	do_lcd_data ' '
	subi r21, -'0'
	do_lcd_data_from_reg r21
	rjmp update_speed_epilogue
print_20s:
	do_lcd_data '2'
	subi r21, 20
	subi r21, -'0'
	do_lcd_data_from_reg r21
	rjmp update_speed_epilogue
print_10s:
	do_lcd_data '1'
	subi r21, 10
	subi r21, -'0'
	do_lcd_data_from_reg r21
	rjmp update_speed_epilogue
update_speed_epilogue:
	do_lcd_data 'm'
	do_lcd_data '/'
	do_lcd_data 's'
	;rcall sleep_50ms			; for buffer, to be able to see
	pop r16
	pop r21
	ret

update_height:
	; r21 had the speed value in m
	; max is 15 m
update_height_prologue:
push r21
push r16
update_height_body:
	do_lcd_command 0b11001110
	lds r21, drone_altitude
	cpi r21, 10
	brsh print_10s_h
	do_lcd_data ' '
	subi r21, -'0'
	do_lcd_data_from_reg r21
	rjmp update_height_epilogue
print_10s_h:
	do_lcd_data '1'
	subi r21, 10
	subi r21, -'0'
	do_lcd_data_from_reg r21
	rjmp update_height_epilogue
update_height_epilogue:
	;rcall sleep_50ms  ; for buffer, to be able to see
	pop r16
	pop r21
	ret

;
; Send a command to the LCD (r16)
;


lcd_command:
	out PORTF, r16
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	ret

lcd_data:
	out PORTF, r16
	lcd_set LCD_RS
	nop
	nop
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	lcd_clr LCD_RS
	ret

lcd_wait:
	push r16
	clr r16
	out DDRF, r16
	out PORTF, r16
	lcd_set LCD_RW
lcd_wait_loop:
	nop
	lcd_set LCD_E
	nop
	nop
        nop
	in r16, PINF
	lcd_clr LCD_E
	sbrc r16, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser r16
	out DDRF, r16
	pop r16
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret

;----------------------------------------------------------------------------------------------------
; Two additional sleep commands were added of longer durations.
sleep_50ms:
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	ret

sleep_100ms:
	rcall sleep_50ms
	rcall sleep_50ms
	ret

sleep_60ms:
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_50ms
	ret

sleep_200ms:
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	ret

sleep_500ms:
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_200ms
	rcall sleep_200ms
	ret
;----------------------------------------------------------------------------------------------------

end_of_file: