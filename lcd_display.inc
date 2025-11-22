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
	do_lcd_command 0b00001100 ; Cursor on, bar, no blink
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


; Displays a message to the lcd reset_screen, relies on print_multiple
; Inputs:
;   @0 = location in program memory of the string
;   @1 = set to 1 to return cursor to home, 0 to leave at current location
; Outputs:
;   Prints a string to the LCD
.macro input_msg_to_display
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


; Used with the Macro input_msg_to_display, prints all characters to the LCD
; Inputs:
;   ZL  = low(string_location<<1)
;   ZH  = high(string_location<<1)
;   r20 = set to 1 to return cursor to home, 0 to leave at current location
; Outputs:
;   Prints a string to the LCD
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
fin_scrolling:
	tst r20
	breq skip_back_to_home
	lcd_back_to_home
skip_back_to_home:
	ret

; Updates the Drone status on the LCD screen. 
; Inputs:
;   reads from data memory drone_state
; Outputs:
;   Updates the drone status on the LCD screen
update_status:
update_status_prologue:
	push r21
	push r16
update_status_body:
	do_lcd_command 0b11000001
	lds r21, drone_state
	cpi r21, 'A'
	brne display_status ; Avoid displaying 'A' as handled elsewhere
	ldi r21, 'F'
display_status:
	do_lcd_data_from_reg r21

	rjmp update_status_epilogue
update_status_epilogue:
	pop r16
	pop r21
	ret

; Updates the drone speed on the LCD screen
; Inputs:
;   reads from data memory drone_speed
; Outputs:
;   Updates the drone speed on the LCD screen
update_speed:
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
	pop r16
	pop r21
	ret

; Updates the drone height on the LCD screen. Maximum height of 15
; Inputs:
;   reads from data memory drone_altitude
; Outputs:
;   Updates the drone altitude on the LCD screen
update_height:
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
	pop r16
	pop r21
	ret


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
