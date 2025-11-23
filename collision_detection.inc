/*
 * collision_detection.inc
 *
 *  Created: 14/11/2025 3:19:45 PM
 *   Author: Cedric
 */ 

; collision_detection.inc

; collision detection for drone navigation
; Checks if drone altitude is sufficient to clear terrain

jmp end_collision_detection

; 
; CHECK_COLLISION
; Checks if drone has crashed into terrain
; Called after each movement
; Logic:
;   - Get current position (x, y)
;   - Get terrain height at that position
;   - Compare altitude with terrain height
;   - If altitude < terrain: CRASH
; Input:
;   drone_current_x = Drone X coordinate
;   drone_current_y = Drone Y coordinate
;   drone_altitude = Drone altitude above ground
;   main_map_start = Terrain height map
; Output:
;   drone_state = Set to 'C' if crashed, unchanged otherwise
check_collision:
check_collision_prologue:
    push r16
    push r20
    push r21
    push r22
    
check_collision_body:

    ; Get current position
    lds r20, drone_current_x     ; X coordinate
    lds r21, drone_current_y     ; Y coordinate
    
    ; Get terrain height at (x, y)
    ; Uses mark's get_value_at_coords function
    ; Input: r20 = x, r21 = y
    ; Output: r22 = height at (x,y)
    call get_value_at_coords
    
    ; Get drone's current altitude
    lds r16, drone_altitude
    
    ; Compare altitude with terrain height
    cp r16, r22                  ; Compare: altitude vs terrain_height
    brsh collision_safe          ; If altitude >= terrain, safe!
    
    ; COLLISION DETECTED
collision_crash:
    ldi r16, 'C'                 ; Set state to 'C' (Crash)
    sts drone_state, r16
    rjmp check_collision_epilogue
    
collision_safe:
    ; No collision, continue normally
    
check_collision_epilogue:
    pop r22
    pop r21
    pop r20
    pop r16
    ret

end_collision_detection:
