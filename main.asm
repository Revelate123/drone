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
.org main_map_memory_start+750
route_size: .byte 1
route_locations: .byte 15*15*3

; Current path waypoints
.org main_map_memory_start+750+675+1
cur_route_size: .byte 1
cur_route_locations: .byte 20

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
    
    jmp main

; ============================================================================
; MAIN SETUP
; ============================================================================
main:
    ; Load map from flash to SRAM
    ldi ZL, low(map_size<<1)
    ldi ZH, high(map_size<<1)
    LPM r16, Z
    sts main_map_size, r16
    sts visibility_map_size, r16
    sts explored_map_size, r16
    
    ldi ZL, low(map<<1)
    ldi ZH, high(map<<1)
    ldi YL, low(main_map_start)
    ldi YH, high(main_map_start)
    mul r16, r16
    mov r16, r0
    
store_map_loop:
    LPM r17, Z+
    st Y+, r17
    dec r16
    brne store_map_loop

	; CRITICAL: Clear explored_map before route generation
	; Without this, timer never worked and generte route always gave me the value of 1 instead of (6) on the hardware
	; It worked fine on the simulator with debugger 
    ldi YL, low(explored_map_start)
    ldi YH, high(explored_map_start)
    ldi r17, 49              ; 7×7 = 49
    ldi r16, 0
clear_explored:
    st Y+, r16
    dec r17
    brne clear_explored

; -----------Added by Tom---------------
	.include "lcd_display.inc"
	reset_screen
; --------------------------------------

    ; Generate observation route
    call generate_route
    
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

; ============================================================================
; MAIN LOOP - WITH CONTINUOUS KEYPAD POLLING (FIXED BRANCHES)
; ============================================================================
main_loop:
    ; ========================================
    ; KEYPAD POLLING LOOP (while waiting for tick)
    ; ========================================
wait_for_tick:
    ; Scan keypad
    call scan_keypad_simple
	call update_height
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

; ============================================================================
; INCLUDES
; ============================================================================
.include "library.inc"
.include "drone_data.inc"
.include "led.inc"
.include "timer.inc"
.include "movement.inc"
.include "keypad.inc"
.include "random_number.inc"
.include "route_generation.inc"
.include "path_between_two_points.inc"
.include "drone_control.inc"
.include "collision_detection.inc" 

; ============================================================================
; FLASH DATA - 7x7 Map
; ============================================================================
.org 0x4000
map_size: .db 7
map: 
    .db 0,0,0,0,0,0,0 ,     0,2,2,2,2,2,0,     0,2,4,4,4,2,0,     0,2,4,6,4,2,0,     0,2,4,4,4,2,0,     0,2,2,2,2,2,0,     0,0,0,0,0,0,0


/* Working Version 0 without timer and keypad
.include "m2560def.inc"

.equ main_map_memory_start = 0x0206
.equ visibility_map_offset = 251
.equ explored_map_offset = 500
; ============================================================================
; DATA SEGMENT
; ============================================================================
.dseg
; We are using register X for reading maps.
; We are using register Y for writing maps.

; Is the main map
.org main_map_memory_start
main_map_size: .byte 1
main_map_start: .byte 15*15


; Is the copy of the map for visibility
.org main_map_memory_start+visibility_map_offset
visibility_map_size: .byte 1
visibility_map_start: .byte 15*15

;
.org main_map_memory_start+explored_map_offset
explored_map_size: .byte 1
explored_map_start: .byte 15*15


; Saved route from route generation function
.org main_map_memory_start+750
route_size: .byte 1
route_locations: .byte 15*15*3 ; Save x, y, z for each location (worst case we look at every location, but very unlikely to happen)


.org main_map_memory_start+750+675+1
cur_route_size: .byte 1
cur_route_locations: .byte 20


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
 
    jmp main

; ============================================================================
; MAIN LOOP
; ============================================================================
main:


    ldi ZL , low(map_size<<1)
    ldi ZH, high(map_size<<1)
    LPM r16, Z
    sts main_map_size, r16
    sts visibility_map_size, r16
    sts explored_map_size, r16
    ldi ZL, low(map<<1)
    ldi ZH, high(map<<1)
    ldi YL, low(main_map_start)
    ldi YH, high(main_map_start)
    mul r16, r16
    mov r16, r0
    store_map_loop:
        LPM r17, Z+
        st Y+, r17
        dec r16
        brne store_map_loop

    nop
    
    call generate_route

	; Uncomment for debugging
    ;nop 

    call initialize_drone
    
	; nop        ; debug point

	; Initialize hovering counter to 0
    ldi r16, 0
    sts hovering_counter, r16
    
  


main_loop:

    ; Check if hovering
    lds r16, hovering_counter
    cpi r16, 0
    breq not_hovering
    
    dec r16
    sts hovering_counter, r16
    rjmp main_loop

not_hovering:
    lds r16, drone_state
    cpi r16, 'D'
    breq route_complete
    
    cpi r16, 'C'
    breq route_crashed
	
    call update_drone_position
    
    ; SAVE arrival status before collision check
    push r16                  ; ? NEW
    call check_collision
    pop r16                   ; ? NEW (restore arrival status)
    
    tst r16                   ; Now checks correct value!
    breq main_loop

    call advance_waypoint
    rjmp main_loop
	
route_complete:
    rjmp route_complete

route_crashed:
    rjmp route_crashed

; ============================================================================
; INCLUDES
; ============================================================================
.include "library.inc"
.include "drone_data.inc"
.include "led.inc"
.include "timer.inc"
.include "movement.inc"
.include "keypad.inc"
.include "random_number.inc"
.include "route_generation.inc"
.include "path_between_two_points.inc"
.include "drone_control.inc"
.include "collision_detection.inc" 

; ============================================================================
; FLASH DATA
; ============================================================================
.org 0x4000
map_size: .db 7
map: 
    .db 0,0,0,0,0,0,0 ,     0,2,2,2,2,2,0,     0,2,4,4,4,2,0,     0,2,4,6,4,2,0,     0,2,4,4,4,2,0,     0,2,2,2,2,2,0,     0,0,0,0,0,0,0
*/

/* Working Version 1 with Timer
;

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
.org main_map_memory_start+750
route_size: .byte 1
route_locations: .byte 15*15*3

; Current path waypoints
.org main_map_memory_start+750+675+1
cur_route_size: .byte 1
cur_route_locations: .byte 20

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

    call init_led
    
    jmp main

; ============================================================================
; MAIN SETUP
; ============================================================================
main:
    ; Load map from flash to SRAM
    ldi ZL, low(map_size<<1)
    ldi ZH, high(map_size<<1)
    LPM r16, Z
    sts main_map_size, r16
    sts visibility_map_size, r16
    sts explored_map_size, r16
    
    ldi ZL, low(map<<1)
    ldi ZH, high(map<<1)
    ldi YL, low(main_map_start)
    ldi YH, high(main_map_start)
    mul r16, r16
    mov r16, r0
    
store_map_loop:
    LPM r17, Z+
    st Y+, r17
    dec r16
    brne store_map_loop

    ; CRITICAL: Clear explored_map before route generation
	; Without this, timer never worked and generte route always gave me the value of 1 instead of (6) on the hardware
	; It worked fine on the simulator with debugger 
	; May be memory overlap or corruption or idk
    ldi YL, low(explored_map_start)
    ldi YH, high(explored_map_start)
    ldi r17, 49              ; 7×7 = 49
    ldi r16, 0

clear_explored:
    st Y+, r16
    dec r17
    brne clear_explored

    ; Generate observation route
    call generate_route
    
    ; Initialize drone at first observation point
    call initialize_drone
    
    ; NOW start timer AFTER everything is ready
	; Uncomment all the code to work on hardware with timer
	; Comment them if you want to see anything on the debugger
    ;call init_simulation_timer
    
    ; Initialize counters
    ldi r16, 0
    sts hovering_counter, r16
    ;sts simulation_tick_flag, r16
    ;sts simulation_counter, r16
    ;sts simulation_counter+1, r16

; ============================================================================
; MAIN LOOP
; ============================================================================
main_loop:
    ; Wait for timer tick (0.5 seconds)
    ;lds r16, simulation_tick_flag
    ;cpi r16, 1
    ;brne main_loop
    
    ; Clear flag
    ;ldi r16, 0
    ;sts simulation_tick_flag, r16
    
    ; Show progress: route_index on LEDs
    ;lds r16, route_index
    ;call show_led_progress
    
    ; Check if hovering
    lds r16, hovering_counter
    cpi r16, 0
    breq not_hovering
    
    ; Decrement hover counter and skip movement
    dec r16
    sts hovering_counter, r16
    rjmp main_loop
    
not_hovering:
    ; Check if done
    lds r16, drone_state
    cpi r16, 'D'
    breq route_complete
    
    ; Check if crashed
    cpi r16, 'C'
    breq route_crashed
    
    ; Move drone one step
    call update_drone_position
    
    ; Check collision (save arrival status first)
    push r16
    call check_collision
    pop r16
    
    ; Check if arrived at waypoint
    tst r16
    breq main_loop
    
    ; Arrived! Advance to next waypoint
    call advance_waypoint
    rjmp main_loop

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

; ============================================================================
; INCLUDES
; ============================================================================
.include "library.inc"
.include "drone_data.inc"
.include "led.inc"
.include "timer.inc"
.include "movement.inc"
.include "keypad.inc"
.include "random_number.inc"
.include "route_generation.inc"
.include "path_between_two_points.inc"
.include "drone_control.inc"
.include "collision_detection.inc" 

; ============================================================================
; FLASH DATA - 7x7 Map
; ============================================================================
.org 0x4000
map_size: .db 7
map: 
    .db 0,0,0,0,0,0,0 ,     0,2,2,2,2,2,0,     0,2,4,4,4,2,0,     0,2,4,6,4,2,0,     0,2,4,4,4,2,0,     0,2,2,2,2,2,0,     0,0,0,0,0,0,0
*/ 
