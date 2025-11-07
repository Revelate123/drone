; A function which takes as input the value stored in R20 and returns to the same register the next random number in the sequence.

.include "m2560def.inc"

jmp EOF

next_rand:
Prologue:
	push YL
	push YH
	push r16
	push r17
	in YL, SPL
	in YH, SPH
	sbiw Y, 8
	; End of prologue

functionBody:

	mov r16, r20
	LSL r16
	EOR r20, r16

	mov r16, r20
	LSR r16
	EOR r20, r16

	mov r16, r20
	LSR r16
	LSR r16
	EOR r20, r16

Epilogue:
	adiw Y, 8
	out SPH, YH
	out SPL, YL
	pop r17
	pop r16
	pop YH
	pop YL
	ret

EOF: