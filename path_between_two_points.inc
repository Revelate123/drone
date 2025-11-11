
jmp end_path_between_two_points

.def route_len = r17
.def cur_x = r18
.def cur_y = r19
.def x1 = r20
.def y1 = r21
.def x2 = r22
.def y2 = r23

; Save point in memory with the points between 2 points (max 20)
; Check if have found the accident location??
; Parameters:
	; r20 - x1
	; r21 - y1
	; r22 - x2
	; r23 - y2
; Will load the path in memory location: cur_route_locations
; Cases:
	; Case 1: curx == x2: 
		; If cur_y < y2, increase cur_y
		; Else, decrease cur_y
	; Case 2: cury == y2
		; If cur_x < x2, increase x2
		; Else, decrease x2
	; Case 3: neither coordinates match - Move both coordinates
		; a. x-coordinate:
			; If cur_x < x2, increase x2
			; Else, decrease x2
		; b. y-coordinate:
			; If cur_y < y2, increase cur_y
			; Else, decrease cur_y
path_between_two_points:
path_between_two_points_prologue:
	push ZL
	push ZH
	push r17
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
path_between_two_points_body:
	; Set up memory location
	; Write to cur_route_locations
	ldi zl, low(cur_route_locations)
	ldi zh, high(cur_route_locations)

	; Set up x and y traversal variables
	mov cur_x, x1
	mov cur_y, y1

	; Initialise route counter
	clr route_len

	path_loop:
		; Store current point in memory
		st Z+, cur_x
		st Z+, cur_y

		; Increment point counter
		inc route_len

		; termination condition
		cp cur_x, x2
		brne continue_path
		cp cur_y, y2
		brne continue_path
		rjmp found_path

		continue_path:
			; CASE 1: Check if x-coordinates are equal
			cp cur_x, x2
			brne check_case_2
			; X-coordinates match - move only in y direction
			rjmp move_y_only

			check_case_2:
				; CASE 2: Check if y-coordinates are equal
				cp cur_y, y2
				brne move_diagonal
				; Y-coordinates match - move only in x direction
				rjmp move_x_only
			
			move_diagonal:
				; CASE 3: Neither coordinate matches - move diagonally
				; Adjust x coordinate
				cp cur_x, x2
				brge decrease_x_diag
				inc cur_x
				rjmp adjust_y_diag
			decrease_x_diag:
				dec cur_x
	
			adjust_y_diag:
				; Adjust y coordinate
				cp cur_y, y2
				brge decrease_y_diag
				inc cur_y
				rjmp path_loop
			decrease_y_diag:
				dec cur_y
				rjmp path_loop

			move_x_only:
				; Move only in x direction
				cp cur_x, x2
				brge decrease_x_only
				inc cur_x
				rjmp path_loop
			decrease_x_only:
				dec cur_x
				rjmp path_loop

			move_y_only:
				; Move only in y direction
				cp cur_y, y2
				brge decrease_y_only
				inc cur_y
				rjmp path_loop
			decrease_y_only:
				dec cur_y
				rjmp path_loop

	found_path:
	; Save route size
	ldi zl, low(cur_route_size)
	ldi zh, high(cur_route_size)
	st Z, route_len

path_between_two_points_epilogue:
	pop r23
    pop r22
    pop r21
    pop r20
    pop r19
    pop r18
    pop r17
    pop ZH
    pop ZL
	ret


end_path_between_two_points: