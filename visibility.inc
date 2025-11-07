; Determines what is visible


.include "m2560def.inc"

.equ visibility_map_offset = 250
.equ explored_map_offset = 500

.def dist = r16




; Calculate distance between two points
; Stores output in register r16
distance(x1, y1, z1, x2, y2, z2):
	cp x1, x2
	brsh next
	swap_two_regs x1, x2
next:
	sub x1, x2
	mul x1, x2


	cp y1, y2
	brlt swapy

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
