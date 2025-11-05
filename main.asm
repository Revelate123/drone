;
; Assignment.asm
;
; Created: 26/10/2025 7:19:05 am
; Author : thoma
;

; We are using unsigned integers
; Replace with your application code
.include "m2560def.inc"
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







