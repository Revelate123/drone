; Determines what is visible


jmp end_visibility

.equ visibility_map_offset = 250
.equ explored_map_offset = 500

.def visL = r14
.def visH = r15
.def distL = r16
.def distH = r17
.def x1 = r18
.def y1 = r19
.def z1 = r20
.def x2 = r21
.def y2 = r22
.def z2 = r23
.def temp = r24


; Calculate distance^2 between two points
; Stores output in register r17:r16
; distance(x1, y1, z1, x2, y2, z2):
; tests:
;     (x1, y1, z1) = (3, 4, 5)
;     (x2, y2, z2) = (15, 7, 15)
;     returns 253
distance:
distance_prologue:
	push YL
	push YH
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	

distance_body:
	clr distL
	clr distH
	square_diff x1, x2
	add distL, x1
	clr temp
	adc distH, temp
	square_diff y1, y2
	add distL, y1
	clr temp
	adc distH, temp
	square_diff z1, z2
	add distL, z1
	clr temp
	adc distH, temp

distance_epilogue:
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop YH
	pop YL
	ret

; Clears the visibility map
; UNTESTED
clear_visibility_map:
clear_visibility_map_prologue:
	push YL
	push YH
	push r18
	push r19

clear_visibility_map_body:
	ldi YL, low(visibility_map_size)
	ldi YH, high(visibility_map_size)
	ld r18, Y+
	ldi r19, 0
cvm_loop:
	st Y+, r19
	dec r18
	brne cvm_loop
clear_visibility_map_epilogue:
	pop r19
	pop r18
	pop YH
	pop YL
	ret

; visibility without blocking
; edits the visibility map 
; takes as parameters the current location
; x1 = r18
; y1 = r19
; z1 = r20
; visL = r14
; visH = r15
check_visible_no_block:
check_visible_no_block_prologue:
	push YL
	push YH
	push x2
	push y2
	push z2
	push r24
	push r25
	push r26
	
check_visible_no_block_body:
		rcall clear_visibility_map
		ldi YL, low(main_map_size)
		ldi YH, high(main_map_size)
		ldi ZL, low(visibility_map_start)
		ldi ZH, high(visibility_map_start)
		ldi r24, low(main_map_size)
		ldi r25, high(main_map_size)
		ld r26, Y+
		mul r26, r26
		mov r26, r0
	cvnbb_loop:
		call get_coords ;stores x2,y2 values
		ld z2, Y+		; stores z2
		call distance	; stores distL:distH using all coords stored in registers
		cp  visL, distL 
		cpc visH, distH
		brlt not_visible
		ldi r21, 1		; r21 no longer required for the rest of loop
		st Z, r21
	not_visible:
		adiw Z, 1
		dec r26
		brne cvnbb_loop

check_visible_no_block_epilogue:
	pop r26
	pop r25
	pop r24
	pop z2
	pop y2
	pop x2
	pop YH
	pop YL
	ret
;check_visible_no_block(current_location, v):
;	clear visiblity map

;	for x in range(15):
;		for y in range(15):
;			ld z, X+
;			c = distance(current_location, x, y, z)
;			if c < v:
;				st 1, Y + visibility_map_offset
;	check_visibile_blocking();
;
;check_visible_blocking(current_location): 
;	ldi YL, low(main_map_start)
;	ldi YH, high(main_map_start)
;	for x in range(15):
;		for y in range(15):
;			ld z, Y+
;			c = distance(current_location, x, y, z)
;			if c < v:
;				st 1, Y + visibility_map_offset


end_visibility: