;;;;;;;;;;;;;;;;;;;;;;;;;;; CODE 1 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 

; The program gets input from keypad and displays its ascii value on LEDs
; Port F is used for keypad, high 4 bits for column selection, low four bits for reading rows. On the board, RF7-4 connect to C3-0, RF3-0 connect to R3-0.
; Port D is used to display the ASCII value of a key.

;----------------------------------------------------------------------------------------------------
; This example code from the lecture has been modified to behave as a function.						|
; The code now stores the key pressed into data memory and then returns from the function call.		|
;----------------------------------------------------------------------------------------------------

.include "m2560def.inc"
	
.def row    =r16		; current row number
.def col    =r17		; current column number
.def rmask   =r18		; mask for current row
.def cmask	=r19		; mask for current column
.def temp1	=r20		
.def temp2  =r21

.equ PORTLDIR =0xF0			; use PortD for input/output from keypad: PF7-4, output, PF3-0, input
.equ INITCOLMASK = 0xEF		; scan from the leftmost column, the value to mask output
.equ INITROWMASK = 0x01		; scan from the bottom row
.equ ROWMASK  =0x0F			; low four bits are output from the keypad. This value mask the high 4 bits.

;------------------------------------------------------------------------------
;Jumps to the end of the file.	
;This prevents code from running when the file is used via .include
jmp skip 
;------------------------------------------------------------------------------


;------------------------------------------------------------------------------
; Prologue to function call. Taken from lecture slides.
get_keys:
	push YL
	push YH
	push r16
	push r17
	push r18
	push r19
	push r20
	push r21
	in YL, SPL
	in YH, SPH
	sbiw Y, 16
	out SPH, YH
	out SPL, YL

	clr r16
	clr r17
	clr r18
	clr r19
	clr r20
	clr r21
;------------------------------------------------------------------------------

RESET:

	ldi temp1, PORTLDIR			; columns are outputs, rows are inputs
	sts	DDRL, temp1
	ser temp1					; PORTC is outputs
	out DDRC, temp1				
	out PORTC, temp1


main:
	ldi cmask, INITCOLMASK		; initial column mask
	clr	col						; initial column
colloop:
	cpi col, 4
	breq main
	sts	PORTL, cmask				; set column to mask value (one column off)
	ldi temp1, 0xFF
delay:
	dec temp1
	brne delay

	lds	temp1, PINL				; read PORTD
	andi temp1, ROWMASK
	cpi temp1, 0xF				; check if any rows are on
	breq nextcol
								; if yes, find which row is on
	ldi rmask, INITROWMASK		; initialise row check
	clr	row						; initial row
rowloop:
	cpi row, 4
	breq nextcol
	mov temp2, temp1
	and temp2, rmask				; check masked bit
	breq convert 				; if bit is clear, convert the bitcode
	inc row						; else move to the next row
	lsl rmask					; shift the mask to the next bit
	jmp rowloop

nextcol:
	lsl cmask					; else get new mask by shifting and 
	inc col						; increment column value
	jmp colloop					; and check the next column

convert:
	cpi col, 3					; if column is 3 we have a letter
	breq letters				
	cpi row, 3					; if row is 3 we have a symbol or 0
	breq symbols

	mov temp1, row				; otherwise we have a number in 1-9
	lsl temp1
	add temp1, row				; temp1 = row * 3
	add temp1, col				; add the column address to get the value
	subi temp1, -'1'			; add the value of character '0'
	jmp convert_end

letters:
	ldi temp1, 'A'
	add temp1, row				; increment the character 'A' by the row value
	jmp convert_end

symbols:
	cpi col, 0					; check if we have a star
	breq star
	cpi col, 1					; or if we have zero
	breq zero					
	ldi temp1, '#'				; if not we have hash
	jmp convert_end
star:
	ldi temp1, '*'				; set to star
	jmp convert_end
zero:
	ldi temp1, '0'				; set to zero

convert_end:
	;out PORTC, temp1			; write value to PORTC

;------------------------------------------------------------------------------
; Instead of jumping back to main, the code stores the key pressed and returns
	st X+, temp1				
;------------------------------------------------------------------------------


;------------------------------------------------------------------------------
; Epilogue to function call, taken from lecture slides.
return:
	adiw Y, 16
	out SPH, YH
	out SPL, YL
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	pop YH
	pop YL
	ret
;------------------------------------------------------------------------------

skip: