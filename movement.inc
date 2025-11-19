;
; movement.inc
;
; Created: 11/11/2025 9:48:23 PM
; Author : Cedric
;

; movement.inc 
; Drone movement system using pre-calculated waypoints


jmp end_movement
	

; UPDATE_DRONE_POSITION 
; Moves drone toward CURRENT WAYPOINT using speed control
; Returns: r16 = 0 (still moving), 1 (arrived at waypoint)

update_drone_position:
update_drone_position_prologue:
    push r17
    push r18
    push r19
    in r19, SREG
    push r19
    
update_drone_position_body:
    ; Check if paused
    lds r16, drone_state
    cpi r16, 'P'
    breq update_pos_no_movement
    
    ; Load current waypoint as target
    call load_current_waypoint_as_target
    
    ; Check if already at this waypoint
    rcall check_at_target_xy
    tst r16
    brne update_pos_arrived

    ; Speed control with accumulator
    lds r16, drone_speed
    lds r17, movement_accumulator
    add r17, r16
    sts movement_accumulator, r17
    
    ; CHANGED: Threshold from 20 to 100
	; 10dm/0.1 sec = 100
    cpi r17, 100                   ; Was: cpi r17, 20
    brlo update_pos_no_movement
    
    ; CHANGED: Subtract threshold from 20 to 100
    subi r17, 100                  ; Was: subi r17, 20
    sts movement_accumulator, r17
    
    ; Move one step toward current waypoint
    rcall move_one_step_to_target
    
    ; Check if arrived after moving
    rcall check_at_target_xy
    tst r16
    brne update_pos_arrived
    
    ldi r16, 0
    rjmp update_drone_position_epilogue
    
update_pos_arrived:
    ldi r16, 0
    sts movement_accumulator, r16
    ldi r16, 1
    rjmp update_drone_position_epilogue
    
update_pos_no_movement:
    ldi r16, 0
    
update_drone_position_epilogue:
    pop r19
    out SREG, r19
    pop r19
    pop r18
    pop r17
    ret

; LOAD_CURRENT_WAYPOINT_AS_TARGET
; Loads the current waypoint from cur_route_locations into drone_target_x/y
load_current_waypoint_as_target:
    push ZL
    push ZH
    push r16
    push r17
    
    ; Get current waypoint index
    lds r16, current_waypoint_index
    
    ; Calculate offset: index × 2 (each waypoint is x,y pair)
    lsl r16                           ; r16 = index × 2
    
    ; Load waypoint address
    ldi ZL, low(cur_route_locations)
    ldi ZH, high(cur_route_locations)
    clr r17
    add ZL, r16
    adc ZH, r17
    
    ; Load waypoint (x, y)
    ld r16, Z+
    sts drone_target_x, r16
    ld r16, Z
    sts drone_target_y, r16
    
    pop r17
    pop r16
    pop ZH
    pop ZL
    ret

; MOVE_ONE_STEP_TO_TARGET
; Moves drone one step toward drone_target_x/y
; directly set position to target (since waypoints are adjacent)
move_one_step_to_target:
    push r16
    
    ; Load target position
    lds r16, drone_target_x
    sts drone_current_x, r16
    
    lds r16, drone_target_y
    sts drone_current_y, r16
    
    pop r16
    ret

; CHECK_AT_TARGET_XY
; Checks if drone's XY position matches target XY
; Returns: r16 = 1 if at target, 0 otherwise
check_at_target_xy:
    push r17
    push r18
    
    lds r17, drone_current_x
    lds r18, drone_target_x
    cp r17, r18
    brne cat_not_at_target
    
    lds r17, drone_current_y
    lds r18, drone_target_y
    cp r17, r18
    brne cat_not_at_target
    
    ldi r16, 1
    rjmp cat_done
    
cat_not_at_target:
    ldi r16, 0
    
cat_done:
    pop r18
    pop r17
    ret

end_movement:
