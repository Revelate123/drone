; assignment_route_generation.asm
;
; Created: 6/11/2025 9:55:34 AM
; Author : mbast
;

jmp end_route_generation

.include "m2560def.inc"
.include "visibility.inc"

; We will save in memory 
; 1. Create a seen vector in data memory. This will let us know 2 things:
	; a. Where to search next - we can just pick the next closest 
	; b. Determine when we can stop the search algorithm (when every location has been visited)
; 2. Have a spot in memory for the points we are picking to visit -> this will be route 

; Functions:
; - Done searching
; - Mark visible locations
; - Closest unseen location

.def fully_explored_flag = r15
.def temp = r16
.def counter = r17
.def route_size_reg = r13 ; TODO: figure out why this for r13 works, but does not if its r18
.def x_idx = r19
.def y_idx = r20
; Do NOT SAVE OVER r21 and r22
.def map_start_address_low = r24
.def map_start_address_high = r25

; Saves counter^2 to counter (r17)
.macro set_up_counter
	mul counter, counter
	mov counter, r0
.endmacro

; Stores in data memory the route as (x, y, h)
	; The algorithm:
		; Set cur_x and cur_y to (0, 0)
		; 1. Add cur_x, cur_y and get_coords(x,y,map) (x, y, z) to the route
		; 2. See what is visible from this location
			; a. Check if we have seen the crash sight, if so, update this location and terminate
		; 3. Update the explored map in memory (essentially visited matrix)
		; 4. Find the next unseen location (update, x_idx, and y_idx) and repeat
	; Stop the route if found is set: this will be set after the visibility function is called
generate_route:
generate_route_prologue:
	; Save all of the registers that I am using
	
	push XL
	push XH
	push YL
	push YH
	push ZL
	push ZH
	push r16
	push r18
	push r19
	push r20
	push r24
	push r25
generate_route_body:
	; Set up registers
	clr route_size_reg
	clr fully_explored_flag
	clr counter

	; Initialise explored map to zeros
	call clear_explored_map
		
	; Set READ from visibility map
	ldi XL, low(visibility_map_size)
	ldi XH, high(visibility_map_size)
	ld counter, X+ ; Get the dimensions for the search

	; Set WRITE to seen map
	ldi YL, low(explored_map_start)
	ldi YH, high(explored_map_start)

	; Set WRITE to route memory location
	ldi ZL, low(route_locations)
	ldi ZH, high(route_locations)

	; GENERATE THE SEARCH PATH
	clr temp
	set_up_counter

	; load explored map address - for get_coords function
	ldi map_start_address_low, low(main_map_start)
	ldi map_start_address_high, high(main_map_start)
	
	; Start our for loop
	route_generation_loop:
		; If the counter is 0, we are done
		cpi counter, 0
		breq done

		; 1. Save current location to the route
			; YH:YL = explored map current location
			; r25:r24 = starting address of explored map
		; Offset for get_coords to work correctly
		subi yl, low(explored_map_offset)
		sbci yh, high(explored_map_offset)
		call get_coords ; get the x and y coordinates of current explored_map location
		; Add the offset back
		subi YL, low(-explored_map_offset) 
		sbci YH, high(-explored_map_offset)
		
		st Z+, r21 ; save x value 
		st Z+, r22 ; save y value
		; TODO: figure out how I want to save the height; for now, just skipping over this memory location
			; This does not handle overflow, should be fine for now
		adiw Z, 1
		
		inc route_size_reg 
		
		; Set memory location to 1 in visibility map
		ldi temp, 1
		st Y, temp

		;  2. Visibility - Set up parameters for check_visible_no_block
		; r21, r22 already have x, y from get_coords
		call setup_and_check_visibility

		; Deal with if crash site is found
		push XL
		push XH
		push YL
		push YH

		ldi XL, low(found_crash_site)
		ldi XH, high(found_crash_site)

		ld temp, X
		cpi temp, 1
		brne no_crash

		handle_crash:
			ldi YL, low(accident_x)
			ldi YH, high(accident_x)

			st Y+, r21
			st Y, r22
			
		no_crash:
		pop YH
		pop YL
		pop XH
		pop XL

		; 3. Update explored map
		call update_explored_map

		; 4. Check if explored map is fully explored
		call check_fully_explored
		ldi temp, 1
		cp fully_explored_flag, temp
		breq done

		; 5. Go to the next unexplored location and loop again
		; increment position of next_unexplored location
		get_next_location_loop:
			adiw Y, 1
			dec counter
			breq done ; if counter is 0, we're done

			ld temp, Y ; load from explored map
			cpi temp, 0 ; if temp is 0, we are at next unexplored location
			breq at_next_location

			rjmp get_next_location_loop


		at_next_location:
		rjmp route_generation_loop
	done:
	
	; Save route size to route_size in DM
	ldi ZL, low(route_size)
	ldi ZH, high(route_size)
	st Z, route_size_reg
	call input_z_values
generate_route_epilogue:
	pop r25
	pop r24
	pop r20
	pop r19
	pop r18
	pop r16
	pop ZH
	pop ZL
	pop YH
	pop YL
	pop XH
	pop XL
	ret

; Sets up parameters and calls check_visible_no_block - TODO: THIS WILL NEED TO BE CHANGED WITH THE FINAL FUNCTION
; Input:
;   r21 = x coordinate
;   r22 = y coordinate
; Output:
;   visibility_map updated
; Modifies:
;   r14, r15, r18, r19, r20 (and whatever check_visible_no_block modifies)
setup_and_check_visibility:
setup_and_check_visibility_prologue:
	push r6
	push XL
	push XH
	push r21  ; preserve these since we use them
	push r22

setup_and_check_visibility_body:
	; Move x, y coordinates to r18, r19
	mov r18, r21  ; x coordinate
	mov r19, r22  ; y coordinate

	; Load the height (z) from main_map at position (x, y)
	ldi XL, low(main_map_size)
	ldi XH, high(main_map_size)
	; r21, r22 still have x, y for get_address_offset
	call get_address_offset ; result in r6

	; Now load from main_map to get height
	ldi XL, low(main_map_start)
	ldi XH, high(main_map_start)
	clr temp
	add XL, r6
	adc XH, temp
	ld r20, X  ; Load height into z1 (r20)

	; Set visibility distance - TODO: currently hard coded, just need to set where we save this
	lds r14, vis_val
	mul r14, r14
	mov r14, r0
	mov r15, r1
	/*
	ldi temp, 3*3
	mov r14, temp
	clr r15 */

	; Now call the function with all parameters set
	; call check_visible_no_block
	call check_visible
setup_and_check_visibility_epilogue:
	pop r22
	pop r21
	pop XH
	pop XL
	pop r6
	ret

; Copies the new seen locations from the visibility map to the explored map
	; Only updates if visibility map = 1 and explored map = 0
; Params : None
; Uses registers:
; r16 = temp
; r17 = counter
; r26 = XL
; r27 = XH
; r28 = YL
; r29 = YH
update_explored_map:
update_explored_map_prologue:
	push XL
	push XH
	push YL
	push YH
	push temp
	push counter
update_explored_map_body:
	; Set READ from visibility map
	ldi XL, low(visibility_map_size)
	ldi XH, high(visibility_map_size)
	ld counter, X+

	; Set WRITE to seen map
	ldi YL, low(explored_map_start)
	ldi YH, high(explored_map_start)

	; Set up counter to decrement
	set_up_counter

	; Loop to update the explored map
	loop:
		; if the counter is at 0, done
		cpi counter, 0
		breq done2

		; Otherwise, perform the update
		ld temp, X+
		cpi temp, 0 ; if visibility[x,y] == 0, skip
		breq not_seen
		; else: update explored matrix to 1
		st Y, temp

		not_seen:
		adiw Y, 1 ; increment Y in either case
		dec counter
		rjmp loop
	done2:
update_explored_map_epilogue:
	pop counter
	pop temp
	pop YH
	pop YL
	pop XH
	pop XL
	ret


; Checks if explored map is fully explored
	; Will set the fully_explored_flag to 1
; Params: None
; Uses registers:
; r16 = temp
; r17 = counter
; r28 = YL
; r29 = YH
check_fully_explored:
check_fully_explored_prologue:
	push YL
	push YH
	push temp
	push counter
check_fully_explored_body:
	ldi YL, low(explored_map_size)
	ldi YH, high(explored_map_size)
	ld counter, Y+
	set_up_counter

	loop1:
		cpi counter, 0 ; if we get to the end of the loop, we can branch to true
		breq true

		ld temp, Y+ ; Checks if current value in explored map is 0, and braches to false if so
		cpi temp, 0
		breq false

		dec counter
		rjmp loop1

	false:
		clr fully_explored_flag
		rjmp done1

	true:
		ldi temp, 1
		mov fully_explored_flag, temp
	done1:
check_fully_explored_epilogue:
	pop counter
	pop temp
	pop YH
	pop YL
	ret


; Fill the route with Z values from the main map
; Parameters: None
; Modifies the route in place
; Uses regs:
; r17 = counter
; r31:30 = point to DRAM location of the route
input_z_values:
input_z_values_prologue:
	push ZL
	push ZH
	push r17
	push r20
	push r21
	push r22
input_z_values_body:
	; Load data memory to read/write
	ldi zl, low(route_size)
	ldi zh, high(route_size)
	ld r17, Z+

	z_loop:
		cpi r17, 0
		breq finished_loading_z_values			

		; Load params for get_value_at_coords
		ld r20, Z+
		ld r21, Z+

		call get_value_at_coords

		st Z+, r22

		dec r17
		rjmp z_loop

	finished_loading_z_values:
input_z_values_epilogue:
	pop r22
	pop r21
	pop r20
	pop r17
	pop ZH
	pop ZL
	ret




; Clears the visibility map
clear_explored_map:
clear_explored_map_prologue:
	push YL
	push YH
	push r18
	push r19

clear_explored_map_body:
	ldi YL, low(explored_map_size)
	ldi YH, high(explored_map_size)
	ld r18, Y+
	mul r18, r18
	mov r18, r0
	ldi r19, 0
cem_loop:
	st Y+, r19
	dec r18
	brne cem_loop
clear_explored_map_epilogue:
	pop r19
	pop r18
	pop YH
	pop YL
	ret

end_route_generation: