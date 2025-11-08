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
	mul r18, r18
	mov r18, r0
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

.macro compute_bound
	mov temp, x2
	subi temp, -@0
	sub temp, x1
	mov lower_bound_X, temp
	mov temp, y2
	subi temp, -@1
	sub temp, y1
	mov lower_bound_Y, temp
.endmacro

.def upper_bound_X = r14
.def upper_bound_Y = r15
.def lower_bound_X = r16
.def lower_bound_Y = r17
.def x_step = r25
.def y_step = r26

; Takes as input the current location as an (x1,y1,z1)
; and blocker location (x2,y2,z2)
calc_blocking_bounds:
calc_blocking_bounds_prologue:
	push YL
	push YH
	push r14
	push r15
	push r16
	push r17
	; push r18
	; push r19
	; push r20
	; push r21
	; push r22
	; push r23
	push r24
	push r25
	push r26

calc_blocking_bounds_body:
	;multiply coordinate system by two
	;there are four corners to each cell.
	;draw a line from the centre of the current cell
	;to the two relevant corners of the blocking cell.
	;Compute two ratios of H/L slopes in 2Dimensions.
	;Compare these ratios with other cells. 
	;If it is within the bounds of these two ratios, it 
	;can be blocked. Else it can't
	; cur_loc, target_loc

	; .def visL = r14
	; .def visH = r15
	; .def distL = r16
	; .def distH = r17
	; .def x1 = r18
	; .def y1 = r19
	; .def z1 = r20
	; .def x2 = r21
	; .def y2 = r22
	; .def z2 = r23
	; .def temp = r24

	mul x1, x1
	mov x1, r0
	mul y1, y1
	mov y1, r0
	;mul z1, z1
	;mov z1, r0
	mul x2, x2
	mov x2, r0
	mul y2, y2
	mov y2, r0
	;mul z2, z2
	;mov z2, r0

	;cur_loc*2
	;target_loc*2



cp x2, x1
brlt x2_lower
	cp y2, y1
	brlt y2_lower_1
	;if x1 < x2 and y1 < y2:
		;top right quadrant
		; upper_bound = target_loc + (-1,1)
		compute_bound -1, 1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound 1, -1
		ldi x_step, 1
		ldi y_step, 1
		rjmp cbbb_end

		; lower_bound = target_loc + (1,-1)
		; lower_bound_X = sub x2+1, x1
		; lower_bound_Y = sub y2-1, y1

	y2_lower_1:
	;elif x1 < x2 and y1 > y2:
		;bottom right quadrant
		; upper_bound = target_loc + (-1,-1)
		; lower_bound = target_loc + (1,1)
		; x_step = 1
		; y_step = -1
		compute_bound -1, -1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound 1, 1
		ldi x_step, 1
		ldi y_step, -1
		rjmp cbbb_end

x2_lower:
	cp y2, y1
	brlt y2_lower_2
	;elif x1 > x2 and y1 > y2:
		; bottom left quadrant
		; upper_bound = target_loc + (1,-1)
		; lower_bound = target_loc + (-1,1)
		; x_step = -1
		; y_step = -1
		compute_bound 1, -1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound -1, 1
		ldi x_step, -1
		ldi y_step, -1
		rjmp cbbb_end
	y2_lower_2:
	;elif x1 > x2 and y1 < y2:
		; top left quadrant
		; upper_bound = target_loc + (1,1)
		; lower_bound = target_loc + (-1,-1)
		; x_step = -1
		; y_step = 1
		compute_bound 1, 1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound -1, -1
		ldi x_step, -1
		ldi y_step, 1
		rjmp cbbb_end

	; elif x1 == x2 and y1 < y2:
	; 	on a straight line up
; 
	; elif x1 == x2 and y1 < y2:
	; 	on a straight line down
; 
	; elif x1 < x2 and y1 == y2:
	; 	on a straight line right
; 
	; elif x1 < x2 and y1 == y2:
	; 	on a straight line left

cbbb_end:
	; Now have two bounds, the current location, blocking start location, x_step, y_step
	; call the next blocking function

calc_blocking_bounds_epilogue:
pop r26
pop r25
pop r24
; pop r23
; pop r22
; pop r21
; pop r20
; pop r19
; pop r18
pop r17
pop r16
pop r15
pop r14
pop YH
pop YL
ret







end_visibility: