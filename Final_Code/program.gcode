; ============================================================================
; SCARA Robot Test Program
; File: program.gcode
; Description: Draw a square, then a circle
; ============================================================================

; Setup
G90             ; Absolute positioning mode
G20             ; Units in inches
G28             ; Move to home position

; Draw a square (2" x 2")
G0 X4 Y4 F5     ; Rapid move to starting corner (4,4)
G1 X6 Y4 F2     ; Draw bottom edge → (6,4)
G1 X6 Y6        ; Draw right edge ↑ (6,6)
G1 X4 Y6        ; Draw top edge ← (4,6)
G1 X4 Y4        ; Draw left edge ↓ close square (4,4)

; Pause between shapes
G4 P1           ; Pause 1 second

; Draw a circle (center at 5,5, radius 1")
G0 X6 Y5 F5     ; Move to circle start (right side)
G2 X6 Y5 I-1 J0 ; Full clockwise circle
                ; I=-1 means center is 1" to the left
                ; J=0 means center is at same Y

; Pause
G4 P1           ; Pause 1 second

; Draw letter "H"
G0 X3 Y3 F5     ; Move to H start
G1 X3 Y5 F2     ; Draw left vertical line
G1 X3 Y4        ; Move to middle
G1 X4 Y4        ; Draw horizontal crossbar
G1 X4 Y5        ; Draw right vertical top
G1 X4 Y3        ; Draw right vertical bottom

; Return home and end
G28             ; Return to home position
M2              ; End program

; ============================================================================
; Command Reference:
; ============================================================================
; G0  - Rapid positioning (pen up)
; G1  - Linear move at feed rate (pen down)
; G2  - Clockwise arc
; G3  - Counter-clockwise arc
; G4  - Dwell/pause (P parameter in seconds)
; G20 - Set units to inches
; G21 - Set units to millimeters
; G28 - Auto-home
; G90 - Absolute positioning
; G91 - Relative positioning
; G92 - Set coordinate offset
; M2  - Program end
;
; Parameters:
; X   - X coordinate
; Y   - Y coordinate
; I   - Arc center X offset
; J   - Arc center Y offset
; F   - Feed rate (inches/sec or mm/sec)
; P   - Dwell time (seconds)
; ============================================================================
