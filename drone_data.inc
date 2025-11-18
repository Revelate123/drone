
; drone_data.inc
; Might need to check/change memory addresses for bigger map 

jmp end_drone_data

.dseg
; Drone data starts at 0x07AD (after cur_route_locations)

drone_current_x: .byte 1          ; 0x07AD
drone_current_y: .byte 1          ; 0x07AE
drone_altitude: .byte 1           ; 0x07AF

drone_target_x: .byte 1           ; 0x07B0
drone_target_y: .byte 1           ; 0x07B1
drone_target_z: .byte 1           ; 0x07B2

drone_speed: .byte 1              ; 0x07B3

drone_state: .byte 1              ; 0x07B4
hovering_counter: .byte 1         ; 0x07B5
hover_complete_flag: .byte 1      ; 0x07B6
search_result: .byte 1            ; 0x07B7
simulation_counter: .byte 2       ; 0x07B8-0x07B9

simulation_tick_flag: .byte 1     ; 0x07BA

movement_accumulator: .byte 1     ; 0x07BB

route_index: .byte 1              ; 0x07BC

; For storing the route location from
; which the accident location is visible
accident_x: .byte 1               ; 0x07BD
accident_y: .byte 1               ; 0x07BE
visibility_distance: .byte 2      ; 0x07BF-0x07C0

led_flash_counter: .byte 1        ; 0x07C1
led_state: .byte 1                ; 0x07C2

last_key_pressed: .byte 1         ; 0x07C3

distance_to_target: .byte 1       ; 0x07C4
current_mountain_height: .byte 1  ; 0x07C5
current_waypoint_index: .byte 1   ; 0x07C6  

found_crash_site: .byte 1 ; TODO: initalise this in main

.cseg

end_drone_data: