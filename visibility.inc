; Determines what is visible


jmp end_visibility

.equ visibility_map_offset = 250
.equ explored_map_offset = 500

.def dist = r16
.def x1 = r17
.def y1 = r18
.def z1 = r19
.def x2 = r20
.def y2 = r21
.def z2 = r22



; Calculate distance^2 between two points
; Stores output in register r16
;distance(x1, y1, z1, x2, y2, z2):

distance_prologue:
	push YL
	push YH
	push r16
	push r17
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25
	in YL, SPL
	in YH, SPH
	sbiw Y, 8
	out SPH, YH
	out SPL, YL

distance:
		



		cp y1, y2
		brlt swapy

distance_epilogue:
	

; visibility without blocking
	; edits the visibility map 
check_visible_no_block(current_location, v):
	clear visiblity map
	ldi YL, low(main_map_start)
	ldi YH, high(main_map_start)
	for x in range(15):
		for y in range(15):
			ld z, X+
			c = distance(current_location, x, y, z)
			if c < v:
				st 1, Y + visibility_map_offset
	check_visibile_blocking();

check_visible_blocking(current_location): 
	ldi YL, low(main_map_start)
	ldi YH, high(main_map_start)
	for x in range(15):
		for y in range(15):
			ld z, Y+
			c = distance(current_location, x, y, z)
			if c < v:
				st 1, Y + visibility_map_offset


end_visibility: