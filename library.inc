.include "m2560def.inc"

jmp end_lib

load_map:
load_map_prologue:
push ZL
push ZH
push r16
push r17
load_map_body:
    ; Load map from flash to SRAM
    ldi ZL, low(map_size<<1)
    ldi ZH, high(map_size<<1)
    LPM r16, Z
    sts main_map_size, r16
    sts visibility_map_size, r16
    sts explored_map_size, r16
    
    ldi ZL, low(map<<1)
    ldi ZH, high(map<<1)
    ldi YL, low(main_map_start)
    ldi YH, high(main_map_start)
    mul r16, r16
    mov r16, r0
    
store_map_loop:
    LPM r17, Z+
    st Y+, r17
    dec r16
    brne store_map_loop

load_map_epilogue:
pop r17
pop r16
pop ZH
pop ZL
ret


.macro swap_two_regs
	mov r24, @0
	mov @0, @1
	mov @1, r24
.endmacro

; Computes the square difference between two values
; e.g. (x1 - x2)^2 and stores the result in @0 may swap @0 and @1
.macro square_diff
	cp @0, @1
		brsh next
		swap_two_regs @0, @1
	next:
		sub @0, @1
		mul @0, @0
		mov @0, R0
.endmacro

.macro swap_if_smaller
	cp @0, @1
	brsh next_1
	swap_two_regs @0, @1
	next_1:
.endmacro

; Gets the z-value at (x, y) coordinates from main map
; Parameters: 
	; x coord: in r20
	; y coord: in r21
; Returns:
	; z coord in r22
; Uses:
	; r16 - temp
	; r17 - size
	; r18 - counter
get_value_at_coords:
get_value_at_coords_prologue:
	push ZL
	push ZH
	push r16
	push r17
	push r18
get_value_at_coords_body:
	ldi ZL, low(main_map_size)
	ldi ZH, high(main_map_size)
	ld r17, Z+ ; loads size for calculation

	; Calculate offset: Y*size + X
	mul r21, r17 
	add r0, r20 ; add X to result of multiplcation (in r0)
	mov r18, r0
	; Now our offset is located in r0
		; using a loop as ldd has offset <= 63 (we could need to go bigger)
	location_loop:
		cpi r18, 0
		breq found_value

		adiw Z, 1
		dec r18
		rjmp location_loop

	found_value:
	ld r22, Z
get_value_at_coords_epilogue:
	pop r18
	pop r17
	pop r16
	pop ZH
	pop ZL
	ret

; Returns the x and y coordinates of a given map address stored in
; register Y.
; Parameters:
;     Address of coordinate in data memory = YH:YL
; Returns:
;     x in r21
;     y in r22
; Example for 7x7 map:
; ldi YL, low(main_map_start)
; ldi YH, high(main_map_start)
; adiw Y, 8
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
	lds r16, main_map_size
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