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


.macro do_lcd_data
	mov r16, @0
	call lcd_data
	call lcd_wait
.endmacro

;----------------------------------------------------------------------------------------------------
; RESET was changed to the macro reset_screen and the default data displayed was deleted.
;----------------------------------------------------------------------------------------------------
.macro reset_screen
	ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

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
	; uses ZL, ZH, r17, r18, r19 pls do stack init for this
	; clear counter (r17)
	; temp = r18
	; delaycounter (r19)
	ldi ZL, low(@0<<1)
	ldi ZH, high(@0<<1)
	rcall print_multiple
.endmacro

print_multiple:
	push r17
	push r18
	push ZL
	push ZH
	clr r17
print_loop:
	lpm r18, Z+
	tst r18
	breq print_done
	do_lcd_data_from_reg temp
	inc r17						; increase counter
	rjmp print_loop
print_done:
	rcall sleep_500ms
	rcall sleep_500ms
	rcall sleep_500ms
	; if r17 counter is more than 16 (visible LCD) -> scroll
	mov r18, r17
lcd_overflow_check:
	cpi r18, 17
	brlo fin_scrolling
	lcd_shift_left
	rcall sleep_500ms
	dec r18
	rjmp lcd_overflow_check
fin_scrolling:
	rcall sleep_500ms
	lcd_back_to_home
	pop ZH
	pop ZL
	pop r18
	pop r17
	ret

display_route_points:
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
	ldi XL, low(route_size_sram)
	ldi XH, high(route_size_sram)
	; we want to read the next route_size * 3
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
	do_lcd_data '1'				; print 10s, printing '1'
	inc r19						; inc counter for char display	

lower_than_10:
	subi r18, -'0'				; digit to ascii
	do_lcd_data_from_reg r18	; print 1s, printing from r18
	inc r19						; inc counter for char display

	cpi r20, 2					; stage check
	brsh found_xyz

	do_lcd_data ','				; still haven't completed (x,y,z)
	inc r19						; inc counter for char display
	inc r20						; inc counter for stage; find x->y->z
	rjmp display_route_loop		; find next value
	
found_xyz:						; found (x,y,z) complete pair
	do_lcd_data '/'				; print slash for distinction
	inc r19						; inc counter for char display
	clr r20						; clear stage
	cpi r17, 0					; compare with 0, end of route?
	brne display_route_loop		; find next value set
	rcall sleep_500ms

lcd_overflow_check_:
	cpi r19, 17					; compare length of data to max display 0-16 chars
	brlo display_route_epilogue
	lcd_shift_left
	rcall sleep_500ms
	dec r19
	rjmp lcd_overflow_check_
display_route_epilogue:
	rcall sleep_500ms
	lcd_back_to_home
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	pop XH
	pop XL
	ret

display_path_between_two_points:
display_path_2_prologue:
	reset_screen
	push XL
	push XH
	push r16
	push r17	; this is path_size
	push r18	; this is for the content
	push r19	; this is for display_char_counter
	push r20	; this is for x, y stages
				; found x. r20 = 0 || found y. r20 = 1
	ldi XL, low(cur_route_size)								; POSSIBLE MACRO @0
	ldi XH, high(cur_route_size)
	; we want to read the next route_size * 3
	ldi r16, 2	; 2: this is for x, y						; POSSIBLE MACRO @1
	ld r17, X+
	mul r17, r16
	mov r17, r0 ; now r17 contains how much next byte to read
	clr r20
	clr r1
	clr r16
	clr r19

display_path_2_loop:
	ld r18, X+					; value saved in r18; max 15
	dec r17						; route value printed -1

	; print here, max value is 15

	cpi r18, 10
	brlo lower_than_10_			; BRANCH
	do_lcd_data '1'				; print 10s, printing '1'
	inc r19						; inc counter for char display	

lower_than_10_:					; BRANCH
	subi r18, -'0'				; digit to ascii
	do_lcd_data_from_reg r18	; print 1s, printing from r18
	inc r19						; inc counter for char display

	cpi r20, 1					; BRANCH stage check
	brsh found_xy				; BRANCH

	do_lcd_data ','				; still haven't completed (x,y)
	inc r19						; inc counter for char display
	inc r20						; inc counter for stage; find x->y
	rjmp display_path_2_loop	; BRANCH find next value
	
found_xy:						; BRANCH found (x,y) complete pair
	do_lcd_data '/'				; print slash for distinction
	inc r19						; inc counter for char display
	clr r20						; clear stage
	cpi r17, 0					; compare with 0, end of route?
	brne display_path_2_loop	; BRANCH find next value set
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
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	pop XH
	pop XL
	ret


update_status:
	; r21 includes the status
	; r21 = 0 Fly
	; r21 = 1 Hover/Pause
	; r21 = 2 Crash
update_status_prologue:
	do_lcd_command 0b11000001
	push r21
update_status_body:
	cpi r21, 1
	brsh pause_or_crash
	do_lcd_data 'F'
	rjmp update_status_epilogue
pause_or_crash:
	cpi r21, 2
	breq crash
	do_lcd_data 'P'
	rjmp update_status_epilogue
crash:
	do_lcd_data 'C'
update_status_epilogue:
	pop r21
	ret

update_speed:
	; r21 had the speed value in dm/s
	; max is 29 m/s
update_speed_prologue:
	do_lcd_command 0b11000110
	push r21
	cpi r21, 20
	brsh print_20s
	cpi r21, 10
	brsh print_10s
	do_lcd_command 0b11000111
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
	rcall sleep_50ms			; for buffer, to be able to see
	pop r21
	ret

update_height:
	; r21 had the speed value in m
	; max is 15 m
update_height_prologue:
	do_lcd_command 0b11001110
	push r21
	cpi r21, 20
	brsh print_20s_h
	cpi r21, 10
	brsh print_10s_h
	do_lcd_command 0b11001111
	subi r21, -'0'
	do_lcd_data_from_reg r21
	rjmp update_height_epilogue
print_20s_h:
	do_lcd_data '2'
	subi r21, 20
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
	rcall sleep_50ms			; for buffer, to be able to see
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

sleep_200ms:
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	ret
;----------------------------------------------------------------------------------------------------

end_of_file: