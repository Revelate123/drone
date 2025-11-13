; ============================================================================
; movement.inc
; ============================================================================
; Drone movement system with diagonal-first strategy
; ============================================================================

jmp end_movement

; ============================================================================
; UPDATE_DRONE_POSITION
; Moves drone toward target using diagonal-first strategy
; Returns: r16 = 0 (still moving), 1 (arrived at target)
; ============================================================================
update_drone_position:
update_drone_position_prologue:
    push r17
    push r18
    push r19
    in r19, SREG
    push r19
    
update_drone_position_body:
    ; CHECK IF PAUSED - ADD THIS!
    lds r16, drone_state
    cpi r16, 'P'
    breq update_pos_no_movement
    
    rcall check_at_target_xy
    tst r16
    brne update_pos_arrived
    
    lds r16, drone_speed
    lds r17, movement_accumulator
    add r17, r16
    sts movement_accumulator, r17
    
    cpi r17, 20
    brlo update_pos_no_movement
    
    subi r17, 20              ; FIXED: was 10, should be 20!
    sts movement_accumulator, r17
    
    rcall move_diagonal_first
    
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

; ============================================================================
; MOVE_DIAGONAL_FIRST
; Diagonal-first movement strategy from hints document
; ============================================================================
move_diagonal_first:
mdf_prologue:
    push r16
    push r17
    push r18
    push r19
    
mdf_body:
    lds r16, drone_target_x
    lds r17, drone_current_x
    sub r16, r17
    mov r18, r16                  ; r18 = dx
    
    lds r16, drone_target_y
    lds r17, drone_current_y
    sub r16, r17
    mov r19, r16                  ; r19 = dy
    
    tst r18
    breq mdf_move_y_only
    
    tst r19
    breq mdf_move_x_only
    
    rjmp mdf_move_diagonal
    
mdf_move_x_only:
    tst r18
    brmi mdf_dec_x
    
    lds r16, drone_current_x
    inc r16
    sts drone_current_x, r16
    rjmp mdf_epilogue
    
mdf_dec_x:
    lds r16, drone_current_x
    dec r16
    sts drone_current_x, r16
    rjmp mdf_epilogue
    
mdf_move_y_only:
    tst r19
    brmi mdf_dec_y
    
    lds r16, drone_current_y
    inc r16
    sts drone_current_y, r16
    rjmp mdf_epilogue
    
mdf_dec_y:
    lds r16, drone_current_y
    dec r16
    sts drone_current_y, r16
    rjmp mdf_epilogue
    
mdf_move_diagonal:
    tst r18
    brmi mdf_diag_dec_x
    
    lds r16, drone_current_x
    inc r16
    sts drone_current_x, r16
    rjmp mdf_diag_y
    
mdf_diag_dec_x:
    lds r16, drone_current_x
    dec r16
    sts drone_current_x, r16
    
mdf_diag_y:
    tst r19
    brmi mdf_diag_dec_y
    
    lds r16, drone_current_y
    inc r16
    sts drone_current_y, r16
    rjmp mdf_epilogue
    
mdf_diag_dec_y:
    lds r16, drone_current_y
    dec r16
    sts drone_current_y, r16
    
mdf_epilogue:
    pop r19
    pop r18
    pop r17
    pop r16
    ret

; ============================================================================
; CHECK_AT_TARGET_XY
; Checks if drone's XY position matches target XY
; Returns: r16 = 1 if at target, 0 otherwise
; ============================================================================
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