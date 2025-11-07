.include "m2560def.inc"

.macro swap_two_regs
	mov temp @0
	mov @0, @1
	mov @1, temp
.endmacro

.macro square_diff
	cp @0, @1
		brsh next
		swap_two_regs @0, @1
	next:
		sub @0, @1
		mul @0, @0
		mov @0, R0
.endmacro

get_value_at_coords:
	@0 - map location
	@1 - x
	@2 - y
	returns memory_location

