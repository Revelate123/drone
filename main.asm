;
; Assignment.asm
;
; Created: 26/10/2025 7:19:05 am
; Author : thoma
;

; We are using unsigned integers
; Replace with your application code
.include "m2560def.inc"
.include "library.inc"
.include "random_number.inc"


; We are using register X for reading maps.
; We are using register Y for writing maps.

; Is the main map
.dseg
.org 0
main_map_size: .byte 1
main_map_start: .byte 15*15



; Is the copy of the map for visibility
.org 250
visibility_map_size: .byte 1
visibility_map_start: .byte 15*15

;
.org 500
explored_map_size: .byte 1
explored_map_start: .byte 15*15


; Saved route from route generation function
.org 750
route_size: .byte 1
route_locations: .byte 15*15*3 ; Save x, y, z for each location (worst case we look at every location, but very unlikely to happen)

.cseg
.org 0x1000
main:
; Map from project brief
map_size: .db 7*7
map: .db 0,0,0,0,0,0,0 , 0,2,2,2,2,2,0 , 0,2,4,4,4,2,0 , 0,2,4,6,4,2,0 , 0,2,4,4,4,2,0 ,  0,2,2,2,2,2,0 , 0,0,0,0,0,0,0

ldi ZL , low(map_size<<1)
ldi ZH, high(map_size<<1)
LPM r16, Z
sts main_map_size, r16
ldi ZL, low(map<<1)
ldi ZH, high(map<<1)
ldi YL, low(main_map_start)
ldi YH, high(main_map_start)
store_map_loop:
	LPM r17, Z+
	st Y+, r17
	dec r16
	brne store_map_loop




