.include "m2560def.inc"

jmp end_lib

.macro swap_two_regs
	mov temp, @0
	mov @0, @1
	mov @1, temp
.endmacro

; Computes the square difference between two values
; e.g. (x1 - x2)^2 and stores the result in @0
.macro square_diff
	cp @0, @1
		brsh next
		swap_two_regs @0, @1
	next:
		sub @0, @1
		mul @0, @0
		mov @0, R0
.endmacro


; Returns the x and y coordinates of a given map address stored in
; register Y.
; Parameters:
;     Address of coordinate in data memory = YH:YL
;     map_start = r25:r24
; Returns:
;     x in r21
;     y in r22
; Example for 7x7 map:
; ldi YL, low(main_map_start)
; ldi YH, high(main_map_start)
; adiw Y, 8
; ldi r24, low(main_map_size)
; ldi r25, high(main_map_size)
; call get_coords
; 
; Output:
; r21 = 1
; r22 = 1
get_coords:
get_coords_prologue:
	push YL
	push YH
	push ZL
	push ZH
	push r23
	push r24
	push r25
get_coords_body:
	ldi r24, low(main_map_size)
	ldi r25, high(main_map_size)
	mov ZL, r24
	mov ZH, r25
	adiw r25:r24, 1
	sub YL, r24
	sbc YH, r25
	clr r22
			
	gc_loop:
	inc r22
	ld r23, Z
	mul r22, r23
	mov r23, r0
	cp  YL, r23
	brsh gc_loop
	dec r22
	; Now y is correct in r22
	ld r23, Z
	mul r22, r23
	sub YL, r0
	; now YL contains just the x coord
	mov r21, YL
get_coords_epilogue:
	pop r25
	pop r24
	pop r23
	pop ZH
	pop ZL
	pop YH
	pop YL
	ret
	
; Loads offset for coordinates
; given in registers r21, r22 and where XH:XL are start of
; map_size.
; Stores offset in r6
get_address_offset:
get_address_prologue:
	push r16
get_address_body:
	clr r6
	lds r16, main_map_start
	mul r22, r16 ; multiply y by row length
	add r6, r0
	add r6, r21 ; add the x step
get_address_epilogue:
	pop r16
	ret

	

;get_value_at_coords:
;	@0 - map location
;	@1 - x
;	@2 - y
;	returns memory_location

end_lib: