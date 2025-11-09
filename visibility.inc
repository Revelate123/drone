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
.def edge_of_map = r26

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
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25
	push r26

;multiply coordinate system by two
;there are four corners to each cell.
;draw a line from the centre of the current cell
;to the two relevant corners of the blocking cell.
;Compute two ratios of H/L slopes in 2Dimensions.
;Compare these ratios with other cells. 
;If it is within the bounds of these two ratios, it 
;can be blocked. Else it can't
; cur_loc, target_loc
calc_blocking_bounds_body:
	add x1, x1
	add y1, y1
	add z1, z1
	add x2, x2
	add y2, y2
	add z2, z2

cp x2, x1
brlt x2_lower
	cp y2, y1
	brlt y2_lower_1
		;top right quadrant
		compute_bound -1, 1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound 1, -1
		ldi x_step, 1
		ldi edge_of_map, 1
		rjmp cbbb_end

	y2_lower_1:
		;bottom right quadrant
		compute_bound -1, -1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound 1, 1
		ldi x_step, 1
		ldi edge_of_map, -1
		rjmp cbbb_end

x2_lower:
	cp y2, y1
	brlt y2_lower_2
		; bottom left quadrant
		compute_bound 1, -1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound -1, 1
		ldi x_step, -1
		ldi edge_of_map, -1
		rjmp cbbb_end
	y2_lower_2:
		; top left quadrant
		compute_bound 1, 1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound -1, -1
		ldi x_step, -1
		ldi edge_of_map, 1
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
	lsr upper_bound_X
	lsr upper_bound_Y
	lsr lower_bound_X
	lsr lower_bound_Y
	mov r8, x_step
	mov r9, edge_of_map
	; Now have two bounds, the current location, blocking start location, x_step, edge_of_map
	; call the next blocking function

calc_blocking_bounds_epilogue:
pop r26
pop r25
pop r24
pop r23
pop r22
pop r21
pop r20
pop r19
pop r18
pop r17
pop r16
pop r15
pop r14
pop YH
pop YL
ret

.macro init_scan_blocking
ldi XL, low(main_map_size)
	ldi XH, high(main_map_size)
	call get_address_offset ; stored in r6
	ld r2, X+
	add r2, r6
	clr r7
	add XL, r6
	adc XH, r7
	ldi ZL, low(visibility_map_start)
	ldi ZH, high(visibility_map_start)
	add ZL, r6
	adc ZH, r7

; This block of code sets the bounds as either 0 or 1
	cpi x_step, 1
	brne set_min
set_max:
	ldi edge_of_map, 255
	rjmp set_complete
set_min:
	ldi edge_of_map, 0
set_complete:

; store blocking coordinates in different registers
	mov r10, r21 
	mov r11, r22
	mov r12, r23

	mov r3, lower_bound_X
	mov r4, lower_bound_Y

rcall slope
mov r8, r13
mov r9, r14
.endmacro

.macro next_el 
	cpi x_step, 1
	brne dec_YZ
inc_YZ:
	adiw Z, 1
	adiw X, 1
	inc r2
	cp r2, edge_of_map
	brne sfbpb_loop
	rjmp sfbpb_end
dec_YZ:
	sbiw Z, 1
	sbiw X, 1
	dec r2
	brne sfbpb_loop
.endmacro


scan_from_blocking_point:
scan_from_blocking_point_prologue:
	push YL
	push YH
	push r6
	push r16

scan_from_blocking_point_body:
	init_scan_blocking

sfbpb_loop:
	call get_coords ;stores x2,y2 values
	ld r16, Z 
	cpi r16, 0
	breq not_blocked
	rcall check_bounds
	mov r16, r5
	cpi r16, 0
	breq not_blocked
	ld z2, X		; stores z2
	call slope
	call blocked
not_blocked:
	next_el

sfbpb_end:
scan_from_blocking_point_epilogue:
pop r16
pop r6
pop YH
pop YL

; Bounds are stored in registers:
.def low_X = r3
.def low_Y = r4
.def high_X = r14
.def high_Y = r15
.def cur_X = r16
.def cur_Y = r17
; (x1,y1,z1) = drone location
; (x2,y2,z2) = cur location considered
; returns bool to register r5
check_bounds:
check_bounds_prologue:
	push r3
	push r4
	push r14
	push r15
	push r16
	push r17
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
check_bounds_body:
compute_bound 0, 0; bounds stored in r16, r17
; Check less than the upper bound
mul high_Y, cur_X 
mov high_Y, r0
mul high_X, cur_Y
mov high_X, r0
cp high_Y, high_X 
brsh less_than_upper
ldi r24, 0
mov r5, r24
rjmp check_bounds_epilogue
less_than_upper:
mul low_Y, cur_X 
mov low_Y, r0
mul low_X, cur_Y
mov low_X, r0
cp low_X, low_Y
brsh check_bounds_epilogue
ldi r24, 0
mov r5, r24
check_bounds_epilogue:
	pop r24
	pop r23
	pop r22 
	pop r21 
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	pop r15
	pop r14
	pop r4
	pop r3
	ret


.macro compare_slopes
mul @0, @3
mov @0, r0
mul @2, @1
mov @2, r0
cp @2, @0
brlt cs_end
ldi r21, 0
st Z, r21
cs_end:
.endmacro

; Register inputs:
; r8  = height difference drone to block
; r9  = length difference drone to block
; r12 = z block location
; r13 = height difference drone to cur
; r14 = length difference drone to cur
; r20 = z drone
; r23 = z cur
; Z 
; Register outputs;
; None
blocked:
blocked_prologue:
push r8
push r9 
push r12 
push r13
push r14
push r20
push r21
push r23
push ZH
push ZL
blocked_body:
	cp r12, r20
	brsh bb_higher
	;if z_block < z_drone:
		;only locations with lower z values than z_block can be blocked
		;and they will have slopes greater
		cp r23, r12
		brsh blocked_epilogue
		compare_slopes r8, r9, r13, r14
		rjmp blocked_epilogue
bb_higher:
	;else:
		;Any location may be blocked.
		cp r23, r12
		brsh bb_next
		ldi r21, 0
		st Z, r21
		rjmp blocked_epilogue
		;if z_cur < z_drone:
			;it is definitely blocked.
bb_next:
		compare_slopes r13, r14, r8, r9
blocked_epilogue:
pop ZL
pop ZH
pop r23
pop r21
pop r20
pop r14
pop r13
pop r12
pop r9
pop r8




; Calculates the slope between two points
; (x1, y1, z1) and (x2, y2, z2)
; Stores the results:
; r13 = (x1-x2)^2 + (y1-y2)^2
; r14 = (z1-z2)^2
slope:
slope_prologue:
	push x1
	push y1
	push z1
slope_body:
	square_diff x1, x2
	square_diff y1, y2
	add x1, y1
	mov r13, x1
	square_diff z1, z2
	mov r14, z1
slope_epilogue:
	pop z1
	pop y1
	pop z1
	ret

;



end_visibility:



; .macro bounded_next_element
; 	add x2, x_step
; 	cpi x2, 255
; 	breq reset_x
; 	cp x2, r2
; 	breq reset_x 
; 	rjmp cont
; reset_x:
; 	mov x2, x1
; 	add y2, edge_of_map
; 	cpi y2, 255
; 	breq end_of_loop
; 	cpi y, r2
; 	breq end_of_loop
; 	rjmp cont:
; end_of_loop:
; 	ser r2
; cont:
; .endmacro