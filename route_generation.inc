; This file contains functions for calculating the route.
; It provides the interface function generate_route.

jmp end_route_generation

.include "m2560def.inc"
.include "visibility.inc"

.def fully_explored_flag = r15
.def temp = r16
.def counter = r17
.def route_size_reg = r13 
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
; Input:
;	None
; Output:
;	route_locations (memory) - Route saved in route_locations
generate_route:
generate_route_prologue:
	; Save all of the registers that gets used
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
		; Skip over this memory location
		adiw Z, 1
		
		inc route_size_reg 
		
		; Set memory location to 1 in visibility map
		ldi temp, 1
		st Y, temp

		;  2. Visibility - Set up parameters for check_visible_no_block
		; r21, r22 already have x, y from get_coords
		call setup_and_check_visibility

		; Deal with if crash site is found
		lds temp, found_crash_site
		cpi temp, 1
		brne no_crash

		handle_crash:
			lds temp, accident_x
			cpi temp, -1
			brne no_crash

			; Update accident location
			sts accident_x, r21
			sts accident_y, r22
			
		no_crash:
	
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

; Sets up parameters and calls check_visible
; Input:
;   r21 = x coordinate
;   r22 = y coordinate
; Output:
;   visibility_map updated
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

	; Set visibility distance
	lds r14, vis_val
	mul r14, r14
	mov r14, r0
	mov r15, r1

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

; Copies newly visible locations from visibility map to explored map
; Only updates explored map entries that are 0 when visibility map is 1
; Input:
;   visibility_map_size (memory) = size of the map 
;   visibility_map (memory) = current visibility status
;   explored_map (memory) = cumulative exploration status
; Output:
;   explored_map (memory) = updated with new visible locations set to 1
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

; Checks if the explored map is fully explored - all entries are 1
; Input:
;   explored_map_size (memory) = size of the map
;   explored_map (memory) = array of exploration status values
; Output:
;   fully_explored_flag = 1 if all map entries are 1, 0 otherwise
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


; Fills Z coordinates in the route by looking up values from the main map
; Input:
;   route_size (memory) = number of waypoints in route
;   route (memory) = array of (X, Y, Z) with Z values uninitialised
; Output:
;   route (memory) = Z values filled in for each waypoint using get_value_at_coords
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

; Clears the explored map by setting all entries to 0
; Input:
;   explored_map_size (memory) = size of the map (will be squared)
; Output:
;   explored_map (memory) = all entries set to 0
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