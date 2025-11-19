; Assignment.asm
;
; Created: 26/10/2025 7:19:05 am
; Author : thoma
;
; LED functions are for testing without LCD 
.include "m2560def.inc"

.equ main_map_memory_start = 0x0206
.equ visibility_map_offset = 251
.equ explored_map_offset = 500

; ============================================================================
; DATA SEGMENT
; ============================================================================
.dseg

; Main map
.org main_map_memory_start
main_map_size: .byte 1
main_map_start: .byte 15*15

; Visibility map
.org main_map_memory_start+visibility_map_offset
visibility_map_size: .byte 1
visibility_map_start: .byte 15*15

; Explored map
.org main_map_memory_start+explored_map_offset
explored_map_size: .byte 1
explored_map_start: .byte 15*15

; Route from generation

route_size: .byte 1
route_locations: .byte 15*15*3

; Current path waypoints

cur_route_size: .byte 1
cur_route_locations: .byte 50

       ; start+1447
crash_x: .byte 1
crash_y: .byte 1
vis_val: .byte 1                                ; start+1450

; ============================================================================
; CODE SEGMENT
; ============================================================================
.cseg
.org 0x0000
    jmp RESET
.org OC1Aaddr              
    jmp TIMER1_COMPA
.org 0x003A
    jmp DEFAULT
DEFAULT: reti

; ============================================================================
; RESET
; ============================================================================
RESET:
    ldi r16, low(RAMEND)
    out SPL, r16
    ldi r16, high(RAMEND)
    out SPH, r16

    call init_led              ; Initialize LED
    call init_keypad_simple    ; Initialize keypad

	; Added by Mark - initialise accident memory, so we dont override it
	ldi r16, 0
	sts found_crash_site, r16

	ldi r16, -1
	sts accident_x, r16
	sts accident_y, r16

    jmp main


; ============================================================================
; User Input Section
; ============================================================================






; ============================================================================
; MAIN SETUP
; ============================================================================
main:
	
	call load_map
; -----------Added by Tom---------------
	.include "lcd_display.inc"
	.include "route_display_tom.inc"
	reset_screen
; --------------------------------------

; -----------Added by Mo----------------
	.include "keypad.inc"
; --------------------------------------
	.include "accident_location.inc"


	
	; User to input location of accident scene (or *,* for no accident)
	call get_crash_xy
	reset_screen
	; User to input visibility distance
	call get_visibility_value
	; Wait for PB0 button press to begin search path generation
	cbi DDRD, 0
	wait_for_pb0:
		sbic PIND, 0
		rjmp wait_for_pb0
	; Generate observation route
	call generate_route
	reset_screen
	call display_route_points_tom
	; Wait for PB1 to start search
	
	cbi DDRD, 1
	wait_for_pb1:
		call update_display_route ; Important, do not touch registers r17 and r19
		ldi r20, 0xff
		ldi r21, 0xff
		ldi r22, 0x0f
		clr r23
	wait_for_pb1_loop:
		sbis PIND, 1
		rjmp done_waiting_for_pb1
		subi r20, 1
		sbci r21, 0
		sbci r22, 0
		cp r20, r23
		cpc r21, r23
		cpc r22, r23
		brne wait_for_pb1_loop
		rjmp wait_for_pb1
		
done_waiting_for_pb1:
	
	;reset_screen ; Moses commented this so that the scrolling will work
	
	; rcall update_route 
	; ^^^^^ this is to be called, when drone arrived to the next search point
	; ^^^^^ this will update the route display by scrolling, ie
	; ^^^^^ 0,0,0/3,5,0/7,8,0/...
	; ^^^^^ ,0,0/3,5,0/7,8,0/...
	; ^^^^^ 0,0/3,5,0/7,8,0/...
    ; ^^^^^ ,0/3,5,0/7,8,0/...
	; ^^^^^ 0/3,5,0/7,8,0/...
	; ^^^^^ /3,5,0/7,8,0/...
	; ^^^^^ 3,5,0/7,8,0/...
    
    ; Initialize drone at first observation point
    call initialize_drone
    
    ; Starting timer 
    call init_simulation_timer
    
    ; Initialize counters
    ldi r16, 0
    sts hovering_counter, r16
    sts simulation_tick_flag, r16
    sts simulation_counter, r16
    sts simulation_counter+1, r16

	; Testing
	/*
	ldi r16, 2
	sts accident_x, r16
	ldi r16, 1
	sts accident_y, r16
	*/
; ============================================================================
; MAIN LOOP - WITH CONTINUOUS KEYPAD POLLING (FIXED BRANCHES)
; ============================================================================
call handle_observation_point_arrival
main_loop:

    ; ========================================
    ; KEYPAD POLLING LOOP (while waiting for tick)
    ; ========================================
wait_for_tick:
    ; Scan keypad
    call scan_keypad_simple
	call update_height
	call update_speed
	call update_status
    ; ========================================
    ; HANDLE PAUSE (Key 0) - Special case
    ; ========================================
    cpi r16, '0'
    brne not_pause_key
    
    ; Key 0 is pressed - PAUSE (if not crashed/done)
    lds r17, drone_state
    cpi r17, 'C'
    breq check_other_keys
    cpi r17, 'D'
    breq check_other_keys
    
    ; Set to paused
    ldi r17, 'P'
    sts drone_state, r17
    rjmp check_tick
    
not_pause_key:
    ; Key 0 NOT pressed - RESUME if was paused
    lds r17, drone_state
    cpi r17, 'P'
    brne check_other_keys
    
    ; Was paused, now resume
    ldi r17, 'F'
    sts drone_state, r17
    
    ; ========================================
    ; HANDLE OTHER KEYS (Altitude/Speed)
    ; ========================================
check_other_keys:
    ; Check if any key pressed
    cpi r16, 0xFF
    brne other_key_pressed
    
    ; No key - clear last_key
    ldi r17, 0xFF
    sts last_key_pressed, r17
    rjmp check_tick
    
other_key_pressed:
    ; Check if this is a NEW press (not Key 0)
    cpi r16, '0'
    breq check_tick
    
    lds r17, last_key_pressed
    cp r16, r17
    breq check_tick
    
    ; NEW key press - process it
    push r16
    sts last_key_pressed, r16
    call process_key_no_pause
    pop r16
    
    ; Debounce delay
    ldi r18, 50
debounce_loop:
    dec r18
    brne debounce_loop
    
check_tick:
    ; Check if timer tick occurred
    lds r16, simulation_tick_flag
    cpi r16, 1
    brne wait_for_tick
    
    ; ========================================
    ; TICK PROCESSING (once per 0.5 seconds)
    ; ========================================
    
    ; Clear tick flag
    ldi r16, 0
    sts simulation_tick_flag, r16
    
    ; Show progress: route_index on LEDs
    lds r16, route_index
    call show_led_progress
    
    ; Check if hovering
    lds r16, hovering_counter
    cpi r16, 0
    breq not_hovering
    
    ; Decrement hover counter and skip movement
    dec r16
    sts hovering_counter, r16
    rjmp tick_done              ; ? Use rjmp (short jump)
    
not_hovering:
    ; Check if done
    lds r16, drone_state
    cpi r16, 'D'
    breq route_complete
    
    ; Check if crashed
    cpi r16, 'C'
    breq route_crashed

	cpi r16, 'A'
	breq accident_location_found
    
    ; Check if paused - INVERTED CONDITION
    cpi r16, 'P'
    brne continue_flying
    rjmp tick_done              ; ? Paused, skip movement
    
continue_flying:
    ; Move drone one step
    call update_drone_position
    
    ; Check collision (save arrival status first)
    push r16
    call check_collision
    pop r16
    
    ; Check if arrived at waypoint
    tst r16
    brne waypoint_arrived
    rjmp tick_done              ; ? Not arrived yet
    
waypoint_arrived:
	call update_display_route1
    ; Arrived! Advance to next waypoint
    call advance_waypoint
    
tick_done:
call update_status
    jmp main_loop               ; ? Absolute jump back to top

; ============================================================================
; END STATES
; ============================================================================
route_complete:
    ; Show all LEDs for completion
    ldi r16, 8
    call show_led_progress
    rjmp route_complete

route_crashed:
    ; Flash LEDs continuously
    lds r16, simulation_tick_flag
    cpi r16, 1
    brne route_crashed
    
    ldi r16, 0
    sts simulation_tick_flag, r16
    
    call flash_led_continuous
    rjmp route_crashed

accident_location_found:
	; Show all LEDs for completion
    ldi r16, 8
    call show_led_progress
	call display_crash_location
	rjmp accident_location_end1

accident_location_end1:
	rjmp accident_location_end1

; ============================================================================
; INCLUDES
; ============================================================================
.include "library.inc"
.include "drone_data.inc"
.include "led.inc"
.include "timer.inc"
.include "movement.inc"
.include "random_number.inc"
.include "route_generation.inc"
.include "path_between_two_points.inc"
.include "drone_control.inc"
.include "collision_detection.inc" 

; ============================================================================
; FLASH DATA - 7x7 Map
; ============================================================================
map_size: .db 15
map: 
    .db 5,5,7,3,6,7,8,8,13,5,4,7,1,14,11,4,11,7,14,8,11,7,6,3,1,8,4,9,10,12,8,9,7,1,9,0,9,7,5,13,2,4,3,10,13,5,5,1,14,8,0,11,13,7,7,13,6,9,14,2,1,5,0,14,2,14,7,12,0,10,8,10,14,6,3,7,5,2,6,9,7,5,1,10,12,13,9,14,11,9,5,3,12,2,10,2,14,0,0,3,9,5,10,14,10,10,0,10,2,13,11,7,10,10,0,13,0,7,14,12,5,3,6,7,13,2,13,11,2,9,14,9,6,1,4,5,7,12,14,13,6,0,0,2,6,0,7,12,7,3,11,5,10,8,1,5,7,6,9,9,2,8,13,14,1,4,14,0,4,1,0,2,1,7,12,14,9,11,7,2,12,7,1,14,6,6,4,9,1,6,3,0,2,11,0,5,3,1,6,7,12,8,3,0,1,8,8,3,2,11,10,5,11,9,3,4,1,8,2,4,2,10,12,5,14
