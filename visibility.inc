; This file contains functions for calculating the visible map locations.
; It provides the interface functions check_visible.

jmp end_visibility

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


; Calculates distance^2 between two points (x1,y1,z1) , (x2,y2,z2)
; Input:
;   r18 = x1
;   r19 = y2
;   r20 = z1
;   r21 = x2
;   r22 = y2
;   r23 = z2
; Output:
;   distance^2 stored in r17:r16
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
; Input:
;   None
; Output:
;   visibility_map set to all 0's
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

; Computes the visible locations from (x1,y1,z1) with visibility (visH:visL)
; without considering blocking
; Input:
;   r18 = x1
;   r19 = y1
;   r20 = z1
;   r14 = visL
;   r15 = visH
; Output:
;    visbility_map updated
check_visible_no_block:
check_visible_no_block_prologue:
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
	push r27
	push ZH
	push ZL
check_visible_no_block_body:
	rcall clear_visibility_map
	ldi YL, low(main_map_size)
	ldi YH, high(main_map_size)
	ldi ZL, low(visibility_map_start)
	ldi ZH, high(visibility_map_start)
	ld r26, Y+			; Map size stored in r26, note this conflicts with register pair X
	mul r26, r26
	mov r26, r0			; Square map size and store in r26, used as a counter to visit all cells.
cvnbb_loop:
	call get_coords		; stores x2,y2 values
	ld z2, Y+			; stores z2
	call distance		; stores distL:distH using all coords stored in registers
	cp  visL, distL		; Compare the visibility with the current distance
	cpc visH, distH
	brlt not_visible
	ldi r21, 1			; Note, r21 no longer required for rest of function.
	st Z, r21			; Mark location in map as visible.
not_visible:
	adiw Z, 1			; Increment Z to point to next cell.
	dec r26				
	brne cvnbb_loop
check_visible_no_block_epilogue:
	pop ZL
	pop ZH
	pop r27
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


; Computes the visible locations from (x1,y1,z1) with visibility (visH:visL). 
; Input:
;   r18 = x1
;   r19 = y1
;   r20 = z1
;   r14 = visL
;   r15 = visH
; Output:
;    visbility_map updated
check_visible:
check_visible_prologue:
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
	push r26
check_visible_body:
	rcall check_visible_no_block
	ldi YL, low(main_map_size)
	ldi YH, high(main_map_size)
	ld r26, Y+			; Map size stored in r26, note this conflicts with register pair X
	mul r26, r26
	mov r26, r0			; Square map size and store in r26, used as a counter to visit all cells.
cvb_loop:
	call get_coords		; stores x2,y2 values
	ld z2, Y+			; stores z2
	rcall scan_from_blocking_point
	dec r26				
	brne cvb_loop
	call check_for_accident
check_visible_epilogue:
	pop r26
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


; Checks whether the accident location was visible in the visibility_map
; Input:
; crash_x (variable in data memory)
; crash_y (variable in data memory)
; Output:
;    If accident visible sets found_crash_site = 1
;    Otherwise no effect
check_for_accident:
check_for_accident_prologue:
push r16
push r17
push r18
push x2
push y2
push YL
push YH
check_for_accident_body:
	ldi YL, low(visibility_map_size)
	ldi YH, high(visibility_map_size)
	ld r16, Y+
	mul r16, r16
	mov r16, r0
	lds r17, crash_x
	lds r18, crash_y
cfa_loop:
	call get_coords_visibility
	ld z2, Y+
	cpi z2, 0
	breq not_found
	cp r17, x2
	brne not_found
	cp r18, y2
	breq cfa_found
not_found:
	dec r16
	brne cfa_loop
	rjmp check_for_accident_epilogue
cfa_found:
	ldi r16, 1
	sts found_crash_site, r16
check_for_accident_epilogue:
pop YH
pop YL
pop y2
pop x2
pop r18
pop r17
pop r16
ret


.def lower_bound_X = r16
.def lower_bound_Y = r17
; Computes horizontal plane bounds and stores results in registers lower_bound_X and lower_bound_Y
; Takes signed integer inputs, @0 and @1 which are added to x2 and y2 prior to calculation. 
; Bounds are stored as |x1 - (x2 + @0)| and |y1 - (y2 + @1)|
; E.g. To compute a straight line between two points, set @0 = 0 and @1 = 0.
; Inputs:
;   @0 = x offset (integer)
;   @1 = y offset (integer)
; Ouputs:
;   r16 = lower_bound_X
;   r17 = lower_bound_Y
.macro compute_bound
	;Store conflicts to the stack
	push x1			
	push y1			
	push r23	
	; r23 used as temp register for subtraction.	
	mov r23, x2
	subi r23, -@0
	swap_if_smaller r23, x1 ; Registers must be swapped in some cases to prevent overflow.
	sub r23, x1				; Compute difference in X
	mov lower_bound_X, r23
	mov r23, y2
	subi r23, -@1
	swap_if_smaller r23, y1
	sub r23, y1				; Computes difference in Y
	mov lower_bound_Y, r23
	pop r23
	pop y1
	pop x1
.endmacro


.def upper_bound_X = r14
.def upper_bound_Y = r15
.def lower_bound_X = r16
.def lower_bound_Y = r17
.def x_step = r25
.def y_step = r24
; Takes as input the current location as an (x1,y1,z1)
; and blocking cell location (x2,y2,z2) and computes upper and lower boundary slopes.
; Explanation:
; Boundary slopes are computed assuming that there are four corners to each cell.
; By drawing two lines from the centre of the current cell to the two corners which give the
; maximum difference in slope and then extrapolating beyond, a "shadow" is cast. 
; Any cells falling within these two boundary lines are within the shadow, and can potentially be blocked.
; To compute the two lines, the coordinate system is multiplied by 2 to allow for "half-steps" to the 
; corners of the cells. Then the "half-steps" are applied depending on the location of the drone and the 
; cell which is casting the "shadow". This causes there to be 9 total cases of blocking:
; a) Four quadrants, which cast diagonal shadows into their quadrants.
; b) Four orthoganals, which cast shadows in a straight line. e.g. along the X or Y axis.
; c) The blocking cell and drone are in the same location.
; Inputs:
;   r18 = x1
;   r19 = y2
;   r20 = z1
;   r21 = x2
;   r22 = y2
;   r23 = z2
; Outputs:
;   r8  = x_step
;   r9  = edge_of_map
;   r13 = y_step
;   r14 = upper_bound_X
;   r15 = upper_bound_Y
;   r16 = lower_bound_X
;   r17 = lower_bound_Y
calc_blocking_bounds:
calc_blocking_bounds_prologue:
	push YL
	push YH
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25
	push r26

calc_blocking_bounds_body:
	lsl x1
	lsl y1
	lsl z1
	lsl x2
	lsl y2
	lsl z2
	ldi XL, low(main_map_size)
	ldi XH, high(main_map_size)
	ld r2, X+ ; used to compare with map bounds
cp x2, x1
breq x2_eq_x1_jump ; Jump to orthoganal cases
brlt x2_lower_jmp
	cp y2, y1
	breq y1_eq_y2_jump ; Jump to orthoganal cases
	brlt y2_lower_1
;-------top right quadrant-----------------------------
		compute_bound -1, 1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound 1, -1
		ldi x_step, 1
		ldi y_step, 0
		mul r2, r2
		mov r9, r0 ; set final step to be n*n-1
		dec r9
		rjmp cbbb_end
;-------jump platforms----------------------------------
x2_lower_jmp:
rjmp x2_lower
x2_eq_x1_jump:
jmp x2_eq_x1
y1_eq_y2_jump:
jmp y1_eq_y2
	y2_lower_1:
;-------bottom right quadrant-------------------------
		compute_bound -1, -1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound 1, 1
		ldi x_step, 1
		ldi y_step, -2
		clr r9
		add r9, r2 
		dec r9; final step is 6
		rjmp cbbb_end
	
x2_lower:
	cp y2, y1
	breq y1_eq_y2_jump2 ; Jump to orthoganal cases
	brlt y2_lower_2
;-------top left quadrant------------------------------
		compute_bound 1, 1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound -1, -1
		ldi x_step, -1
		ldi y_step, 2
		mul r2, r2
		sub r0, r2
		mov r9, r0 ; set final step as n*(n-1)
		rjmp cbbb_end
y1_eq_y2_jump2:
jmp y1_eq_y2
	y2_lower_2:
;-------bottom left quadrant---------------------------
		compute_bound 1, -1
		mov upper_bound_X, lower_bound_X
		mov upper_bound_Y, lower_bound_Y
		compute_bound -1, 1
		ldi x_step, -1
		ldi y_step, 0
		clr r9
		rjmp cbbb_end

;-------Orthoganal directions-------------------------
;-------Vertical--------------------------------------
	x2_eq_x1:
		clr upper_bound_X
		clr upper_bound_Y
		clr lower_bound_X
		clr lower_bound_Y
		clr x_step
		cp y1, y2
		brlt up
down:
		ldi y_step, -1
		clr r9
		add r9, x1
		lsr r9
		rjmp cbbb_end
up:
		ldi y_step, 1
		mul r2, r2
		sub r0, r2
		mov r9, x1
		lsr r9
		add r0, r9
		mov r9, r0
		rjmp cbbb_end
;-------Horizontal-------------------------------------
y1_eq_y2:
		clr upper_bound_X
		clr upper_bound_Y
		clr lower_bound_X
		clr lower_bound_Y
		clr y_step
		cp x1, x2
		brlt right
left:
		ldi x_step, -1
		lsr y2
		mul y2, r2
		lsl y2
		mov r9, r0
		rjmp cbbb_end
right:
		ldi x_step, 1
		mov r9, r2
		lsr y2
		mul y2, r2
		lsl y2
		add r9, r0
		dec r9
		rjmp cbbb_end

cbbb_end:
	mov r8, x_step
	mov r13, y_step
	; Now have two bounds, the current location, blocking start location, x_step, edge_of_map, y_step

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
pop YH
pop YL
ret

.def y_step = r13
.def  x_step = r8
.def  edge_of_map = r9
; Moves registers X and Z to the next location on the map dependant on x_step,
; y_step, and edge_of_map. For use with scan_from_blocking_point.
; Inputs:
;   r31 = ZH 
;   r30 = ZL
;   r27 = XH
;   r26 = XL
;   r22 = y2
;   r21 = x2
;   r17 = temp
;   r13 = y_step
;   r9  = edge_of_map
;   r8  = x_step
;   r2  = current linear position in map
; Outputs:
;   r31 = ZH 
;   r30 = ZL
;   r27 = XH
;   r26 = XL  
;   r24 = flag set when reached edge_of_map
next_el:
next_el_prologue:
	push r8
	push r9
	push r13
	push r16
	push r17
	push r18
	push YH
	push YL
next_el_body:
	ldi YL, low(main_map_size)
	ldi YH, high(main_map_size)
	ld r17, Y ; Get map size
	mul y2, r17 
	add r0, x2
	cp r0, edge_of_map ; Check if last position in map has been reached.
	breq ne_fin_loop
	clr r16
;---------Check for special case x_step == 0---------
	cp x_step, r16
	breq add_columns
;---------add x_step-----------------
	cp x_step, r16
	brge cont
	dec r16
cont:
	add ZL, x_step
	adc ZH, r16
	add XL, x_step
	adc XH, r16
	add r2, x_step
	add x2, x_step
; compare x2 to the row width, if greater then add x_step*row_width
; works for both increment and decrement via overflow.
	cp x2, r17
	brlo ne_cont_loop ; If not beyond a row start/end then y_step does not need adding.
	mul x_step, r17
	add x2, r0 ; reset back to row start/end. Adds either -size or +size
;--------add y_step------------------
add_columns:
	mov r18, y_step
	muls r18, r17
	add ZL, r0
	adc ZH, r1
	add XL, r0
	adc XH, r1
	add r2, r0
	rjmp ne_cont_loop
	;---------------------- 
ne_fin_loop:
	clr r24
	rjmp next_el_epilogue

ne_cont_loop:
	ldi r24, 1

next_el_epilogue:
	pop YL
	pop YH
	pop r18
	pop r17
	pop r16
	pop r13
	pop r9
	pop r8
	ret

; This function computes whether individual cells are blocked based on the drone location
; (x1,y1,z1) and a chosen blocking location (x2,y2,z2).
; Inputs:
;   r23 = z_block
;   r22 = y_block 
;   r21 = x_block
;   r20 = z_drone
;   r19 = y_drone
;   r18 = x_drone 
; Outputs:
;    visbility_map updated
.equ block_slope_Z = 1
.equ block_slope_XY = 2
scan_from_blocking_point:
scan_from_blocking_point_prologue:
	push YL
	push YH
	push r2
	push r3
	push r4
	push r5
	push r6
	push r7
	push r8
	push r9
	push r10
	push r11
	push r12
	push r13
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
	push r27
	push r28
	push r29
	push r30
	push r31

scan_from_blocking_point_body:
init_scan_blocking:
	rcall calc_blocking_bounds
	ldi XL, low(main_map_size)
	ldi XH, high(main_map_size)
	call get_address_offset ; stored in r6
	ld r2, X+ ; used to compare with map bounds
	
	clr r7
	add XL, r6 ; move X to the current blocking cell.
	adc XH, r7 
	ldi ZL, low(visibility_map_start)
	ldi ZH, high(visibility_map_start)
	add ZL, r6 ; move Z to the current blocking cell.
	adc ZH, r7
	
set_complete:
	mov r3, lower_bound_X
	mov r4, lower_bound_Y
	mov r12, r23

rcall slope ; stores results in r6 and r7
mov r10, r6
mov r11, r7

sfbpb_loop:
	mov YH, XH
	mov YL, XL
	call get_coords ;stores x2,y2 values based on Y
	ld r16, Z 
	cpi r16, 0 ; Check if cell is already not visible.
	breq out_of_bounds
	rcall check_bounds
	mov r16, r5
	cpi r16, 0
	breq out_of_bounds
	ld z2, X	
	call slope ; stores results in r6 and r7
	call blocked
out_of_bounds:
	call next_el
	cpi r24, 1
	breq sfbpb_loop

sfbpb_end:
scan_from_blocking_point_epilogue:
pop r31
pop r30
pop r29
pop r28
pop r27
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
pop r13
pop r12
pop r11
pop r10
pop r9
pop r8
pop r7
pop r6
pop r5
pop r4
pop r3
pop r2
pop YH
pop YL
ret



.def low_X = r3
.def low_Y = r4
.def high_X = r14
.def high_Y = r15
.def cur_X = r16
.def cur_Y = r17
; Checks whether the current location (x2,y2,z2) is within the upper and
; lower bounds relative to the drone location (x1,y1,z1)
; Inputs:
;   r3  = lower bound X
;   r4  = lower bound Y
;   r14 = upper bound X
;   r15 = upper bound Y
;   r16 = current slope X
;   r17 = current slope Y
; Outputs:
;    If within bounds sets r5 = 1
;    Otherwise sets        r5 = 0 
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
clr r5
rjmp check_bounds_epilogue
less_than_upper:
mul low_Y, cur_X 
mov low_Y, r0
mul low_X, cur_Y
mov low_X, r0
cp low_X, low_Y
brsh set_true
clr r5
rjmp check_bounds_epilogue
set_true:
	clr r5
	inc r5
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


; Compares two slopes and sets the visibility_map at the current location to 0 
; if the location is blocked. Each slopes can be represented as Vertical / Horizontal
; Inputs:
;   @0 = Horizontal slope 1
;   @1 = Vertical slope 1
;   @2 = Horizontal slope 2
;   @3 = Vertical slope 2
; Outputs:
;    visbility_map updated
.macro compare_slopes
push r16
push r17
mul @0, @3
mov @0, r0
mov r16, r1
mul @2, @1
mov @2, r0
mov r17, r1
cp  @0, @2
cpc r16, r17
brsh cs_end
ldi r21, 0
st Z, r21
cs_end:
	pop r17
	pop r16
.endmacro

; Checks if a location is blocked by checking slopes relative to the drone 
; position (x1,y1,z1)
; Inputs:
;   r11 = Vertical slope drone to block
;   r10 = Horizontal slope drone to block
;   r12 = z block location
;   r7  = Vertical slope drone to cur
;   r6  = Horizontal slope drone to cur
;   r20 = Drone height
;   r23 = Current height
;   Z   = Location of current in visibility_map
; Register outputs;
; None
blocked:
blocked_prologue:
push r11
push r10 
push r12 
push r7
push r6
push r20
push r21
push r23
push ZH
push ZL
blocked_body:
	cp r12, r20
	brsh bb_higher
	; if z_block < z_drone only locations with lower z values
	; than z_block can be blocked and they will have slopes greater
		cp r23, r12
		brsh blocked_epilogue
		compare_slopes r11, r10, r7, r6
		rjmp blocked_epilogue
bb_higher:
		cp r23, r20
		brsh bb_next
		; If z_current < z_drone and z_drone < z_block then current is blocked.
		ldi r21, 0
		st Z, r21
		rjmp blocked_epilogue
bb_next:
		compare_slopes r7, r6, r11, r10

blocked_epilogue:
pop ZL
pop ZH
pop r23
pop r21
pop r20
pop r6
pop r7
pop r12
pop r10
pop r11
ret


; Calculates the slope between two points (x1, y1, z1) and (x2, y2, z2).
; Slopes are stored as Vertical / Horizontal 
; Inputs:
;   r18 = x1
;   r19 = y2
;   r20 = z1
;   r21 = x2
;   r22 = y2
;   r23 = z2
; Outputs:
;   r6 = (x1-x2)^2 + (y1-y2)^2
;   r7 = (z1-z2)^2
slope:
slope_prologue:
	push x1
	push y1
	push z1
	push x2
	push y2
	push z2
	push r24
slope_body:
	square_diff x1, x2
	square_diff y1, y2
	add x1, y1
	mov r6, x1
	square_diff z1, z2
	mov r7, z1
slope_epilogue:
	pop r24
	pop z2
	pop y2
	pop x2
	pop z1
	pop y1
	pop x1
	ret

end_visibility: