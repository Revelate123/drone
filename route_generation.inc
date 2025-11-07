; assignment_route_generation.asm
;
; Created: 6/11/2025 9:55:34 AM
; Author : mbast
;

; We will save in memory 
; 1. Create a seen vector in data memory. This will let us know 2 things:
	; a. Where to search next - we can just pick the next closest 
	; b. Determine when we can stop the search algorithm (when every location has been visited)
; 2. Have a spot in memory for the points we are picking to visit -> this will be route 

; Functions:
; - Done searching
; - Mark visible locations
; - Closest unseen location

; this will store in memory the route as
generate_route:
	; PROLOGUE
	; What do i need to save? - SREG?, definitely Y register?

	; BODY

	; EPILOGUE