.include "m2560def.inc"

.macro swap_two_regs
	mov temp @0
	mov @0, @1
	mov @1, temp
.endmacro

get_value_at_coords:
	@0 - map location
	@1 - x
	@2 - y
	returns memory_location
	