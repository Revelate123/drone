;
; drone_control.inc
;
; Created: 14/11/2025 1:07:44 PM
; Author : Cedric
;


; Main drone control logic for following waypoint paths
jmp end_drone_control


; INITIALIZE_DRONE
; Sets up drone at first observation point and generates first path
initialize_drone:
    push r16
    push ZL
    push ZH
    
    ; Start at observation point 0
    ldi r16, 0
    sts route_index, r16
    sts current_waypoint_index, r16
    
    ; Load first observation point position
    ldi ZL, low(route_locations)
    ldi ZH, high(route_locations)
    ld r16, Z+                    ; X
    sts drone_current_x, r16
    ld r16, Z+                    ; Y
    sts drone_current_y, r16
    ld r16, Z                     ; Z
    sts drone_altitude, r16
    
    ; Set initial state
    ldi r16, 'F'                  ; Flight mode
    sts drone_state, r16
    
    ; Set initial speed
    ldi r16, 1               ; Can change speed here for testing 
    sts drone_speed, r16

	; Set initial altitude - Debugging with low altitude for now
    ldi r16, 10              ; Low altitude - will crash // High altitude (10) - will safe pass 
    sts drone_altitude, r16
    
    ; Generate path to next observation point
    call generate_path_to_next_observation
    
    pop ZH
    pop ZL
    pop r16
    ret


; GENERATE_PATH_TO_NEXT_OBSERVATION
; Generates waypoints from current position to next observation point
; Calls path_between_two_points

generate_path_to_next_observation:
    push r20
    push r21
    push r22
    push r23
    push ZL
    push ZH
    push r16
    push r17
    
    ; Load current position as start
    lds r20, drone_current_x      ; x1
    lds r21, drone_current_y      ; y1
    
    ; Get next observation point
    lds r16, route_index
    inc r16
    sts route_index, r16
    
    ; Check if finished entire route
    lds r17, route_size
    cp r16, r17
    brsh gpno_route_finished
    
    ; Calculate offset in route_locations: index × 3
    ldi r17, 3
    mul r16, r17
    
    ; Load next observation point
    ldi ZL, low(route_locations)
    ldi ZH, high(route_locations)
    add ZL, r0
    adc ZH, r1
    
    ld r22, Z+                    ; x2
    ld r23, Z+                    ; y2
    ld r16, Z                     ; z2 (target altitude)
    sts drone_target_z, r16
    
    ; Generate path
    call path_between_two_points
    
    ; Reset waypoint index to 0
    ldi r16, 0
    sts current_waypoint_index, r16
    
    pop r17
    pop r16
    pop ZH
    pop ZL
    pop r23
    pop r22
    pop r21
    pop r20
    ret

gpno_route_finished:
    ; Display results
    ldi r16, 'D'                  ; Done state
    sts drone_state, r16
    
    pop r17
    pop r16
    pop ZH
    pop ZL
    pop r23
    pop r22
    pop r21
    pop r20
    ret


; ADVANCE_WAYPOINT
; Moves to next waypoint in current path
; If all waypoints done, handles observation point arrival

advance_waypoint:
    push r16
    push r17
    
    ; Increment waypoint index
    lds r16, current_waypoint_index
    inc r16
    sts current_waypoint_index, r16
    
    ; Check if finished all waypoints (reached observation point)
    lds r17, cur_route_size
    cp r16, r17
    brlo aw_not_done
    
    ; Reached observation point!
    call handle_observation_point_arrival
    
aw_not_done:
    pop r17
    pop r16
    ret


; HANDLE_OBSERVATION_POINT_ARRIVAL
; Called when drone reaches an observation point
; Starts hovering, checks for accident, generates next path

handle_observation_point_arrival:
    push r16
    
     ; Show LED progress
    lds r16, route_index
    call show_led_progress
    
    ; Start hovering
    ldi r16, 10         ; can change
    sts hovering_counter, r16
    
    ; TODO: Check if accident visible here
    ; call check_accident_visible
    
    ; After hovering, generate path to next observation point
    call generate_path_to_next_observation
    
    pop r16
    ret


end_drone_control: