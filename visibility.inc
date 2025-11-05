.include "m2560def.inc"

; Whats reachable?

.def x1 = ..





;distance(x1, y1, z1, x2, y2, z2):
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
	ldi XL, low(main_map_start)
	ldi XH, high(main_map_start)
	for x in range(15):
		for y in range(15):
			ld z, X+
			c = distance(current_location, x, y, z)
			if c < v:
				st 1, X+250 ; confirm that doesnt change X 
	check_visibile_blocking();

check_visible_blocking(current_location): 
	ldi XL, low(main_map_start)
	ldi XH, high(main_map_start)
	for x in range(15):
		for y in range(15):
			ld z, X+
			c = distance(current_location, x, y, z)
			if c < v:
				st 1, X+250 ; confirm that doesnt change X 
