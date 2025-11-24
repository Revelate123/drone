
; drone_data.inc
; Saves locations in data memory for drone data

jmp end_drone_data

.dseg
; Drone data 

drone_current_x: .byte 1         
drone_current_y: .byte 1          
drone_altitude: .byte 1         

drone_target_x: .byte 1          
drone_target_y: .byte 1          
drone_target_z: .byte 1         

drone_speed: .byte 1             

drone_state: .byte 1             
hovering_counter: .byte 1         
hover_complete_flag: .byte 1      
search_result: .byte 1            
simulation_counter: .byte 2      

simulation_tick_flag: .byte 1  

movement_accumulator: .byte 1    

route_index: .byte 1             

; For storing the route location from
; which the accident location is visible
accident_x: .byte 1               
accident_y: .byte 1              
visibility_distance: .byte 2     

led_flash_counter: .byte 1        
led_state: .byte 1               

last_key_pressed: .byte 1         

distance_to_target: .byte 1      
current_mountain_height: .byte 1 
current_waypoint_index: .byte 1  

found_crash_site: .byte 1 

main_crash_site_found: .byte 1

.cseg

end_drone_data:
