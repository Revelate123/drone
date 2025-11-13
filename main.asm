;
; Assignment.asm
;
; Created: 26/10/2025 7:19:05 am
; Author : thoma
;


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

    call init_led
    call init_led_flash
    call init_keypad_simple     ; Initialize keypad on PORTL

  
 

    ;cpi r16, 0
    ;brne error_flash
    ;call init_simulation_timer

    jmp main

error_flash:
    call flash_led_continuous
    rjmp error_flash



; ============================================================================
; MAIN LOOP - WITH KEYPAD CONTROL
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
/*
main_loop:
    ; Scan keypad (non-blocking)
    call scan_keypad_simple
    call process_key
    
    ; Wait for timer tick
    lds r16, simulation_tick_flag
    cpi r16, 0
    breq main_loop
    
    ldi r16, 0
    sts simulation_tick_flag, r16
    
    ; Move drone
    call update_drone_position
    tst r16
    breq no_arrival
    
    ; Arrived -> next waypoint
    lds r16, route_index
    inc r16
    sts route_index, r16


no_arrival:
    ; Show position on LED
    lds r16, drone_current_x
    out PORTC, r16

	
    
    rjmp main_loop
*/
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

; ============================================================================
; FLASH DATA
; ============================================================================
.org 0x4000
map_size: .db 7
map: 
    .db 0,0,0,0,0,0,0 ,     0,2,2,2,2,2,0,     0,2,4,4,4,2,0,     0,2,4,6,4,2,0,     0,2,4,4,4,2,0,     0,2,2,2,2,2,0,     0,0,0,0,0,0,0

	/*






finished:
	rjmp finished
	*/