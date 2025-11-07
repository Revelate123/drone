; Determines what is visible


jmp end_visibility

.equ visibility_map_offset = 250
.equ explored_map_offset = 500

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


clear_visibility_map:
clear_visibility_map_prologue:
	push YL
	push YH
	push r18
	push r19

clear_visibility_map_body:
	ld YL, low(visibility_map_size)
	ld YH, high(visibility_map_size)
	ldd r18, Y+
	ldi r19, 0
cvm_loop:
	st Y+, 0
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

check_visible_no_block:
	rcall clear_visibility_map
	ldi YL, low(main_map_start)
	ldi YH, high(main_map_start)

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