.include "m2560def.inc"

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
;     map_offset = r25:r24
; Returns:
;     x in r21
;     y in r22
get_coords:
get_coords_prologue:
	push YL
	push YH
	push ZL
	push ZH
	push r21
	push r22
	push r23
	push r24
	push r25
get_coords_body:
	mov ZH, YH
	mov ZL, YL
	adiw Y , -map_offset
	sub YL, r24
	sbc YH, r25
	clr r22
			
	loop:
	inc r22
	lpm r23, Z
	mul r22, r23
	mov r23, r0
	cp r23, YL
	brlt loop
	dec r22
	; Now y is correct in r22
	lpm r23, Z
	mul r22, r23
	sub YL, r0
	; now YL contains just the x coord
	mov r21, YL
get_coords_epilogue:
	pop r25
	pop r24
	pop r23
	pop r22
	pop r21
	pop ZH
	pop ZL
	pop YH
	pop YL
	ret
	

;get_value_at_coords:
;	@0 - map location
;	@1 - x
;	@2 - y
;	returns memory_location

