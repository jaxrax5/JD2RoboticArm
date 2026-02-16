"""
Configuration file for SCARA Robot Controller
Adjust these values to match your hardware setup
"""

# ============================================================================
# WORKSPACE CONFIGURATION (8.5" x 11" paper)
# ============================================================================

PAPER_WIDTH_INCHES = 8.5
PAPER_HEIGHT_INCHES = 11.0

# Canvas scaling: pixels per inch
PIXELS_PER_INCH = 50  # 50 pixels = 1 inch (425 x 550 canvas)

CANVAS_WIDTH = int(PAPER_WIDTH_INCHES * PIXELS_PER_INCH)   # 425 pixels
CANVAS_HEIGHT = int(PAPER_HEIGHT_INCHES * PIXELS_PER_INCH)  # 550 pixels

# ============================================================================
# ROBOT ARM CONFIGURATION
# ============================================================================

# Arm segment lengths (in inches)
ARM_L1 = 6.0  # Length of first arm segment (shoulder to elbow)
ARM_L2 = 6.0  # Length of second arm segment (elbow to wrist)

# Maximum reach = L1 + L2 = 12 inches
# Minimum reach = |L1 - L2| = 0 inches (singularity at center)

# Home position (in inches, relative to robot base)
HOME_X = 6.0
HOME_Y = 6.0

# ============================================================================
# SERVO CONFIGURATION
# ============================================================================

# Servo angle limits (degrees)
SERVO1_MIN = 0
SERVO1_MAX = 180
SERVO2_MIN = 0  
SERVO2_MAX = 180

# Servo home positions (degrees)
SERVO1_HOME = 75   # Matches your Arduino code
SERVO2_HOME = 120  # Matches your Arduino code

# Angle offsets for calibration (adjust if robot moves incorrectly)
SERVO1_OFFSET = 0  # Add/subtract degrees to servo 1 commands
SERVO2_OFFSET = 0  # Add/subtract degrees to servo 2 commands

# ============================================================================
# MOTION PARAMETERS
# ============================================================================

# Default feed rate (inches per second)
DEFAULT_FEED_RATE = 2.0

# Maximum feed rate (inches per second)
MAX_FEED_RATE = 10.0

# Step delay for smooth servo movement (milliseconds)
# Lower = faster but jerkier, Higher = slower but smoother
SERVO_STEP_DELAY = 15  # ms between 1-degree steps

# Arc interpolation resolution (segments per inch of arc length)
ARC_SEGMENTS_PER_INCH = 10

# ============================================================================
# SD CARD CONFIGURATION
# ============================================================================

# Output filename for Arduino SD card
SD_OUTPUT_FILE = "moves.txt"

# Format: "angle1,angle2" per line (matches your Arduino code)

# ============================================================================
# GUI CONFIGURATION
# ============================================================================

WINDOW_TITLE = "SCARA Robot Controller - SD Card Mode"
BRUSH_SIZE = 3
DRAW_COLOR = "black"

# ============================================================================
# COORDINATE SYSTEM
# ============================================================================

# Robot base position relative to paper
# (0,0) is at bottom-left corner of paper
# X increases to the right
# Y increases upward

# Canvas coordinate conversion:
# Canvas (0,0) is top-left
# Robot (0,0) is bottom-left
# Therefore: robot_y = CANVAS_HEIGHT - canvas_y

# ============================================================================
# UNITS
# ============================================================================

# Default units (can be changed with G20/G21)
DEFAULT_UNITS = "inches"  # or "mm"

# Conversion factor
MM_PER_INCH = 25.4

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def canvas_to_robot(canvas_x, canvas_y):
    """Convert canvas pixel coordinates to robot workspace inches"""
    robot_x = canvas_x / PIXELS_PER_INCH
    robot_y = (CANVAS_HEIGHT - canvas_y) / PIXELS_PER_INCH
    return robot_x, robot_y

def robot_to_canvas(robot_x, robot_y):
    """Convert robot workspace inches to canvas pixel coordinates"""
    canvas_x = int(robot_x * PIXELS_PER_INCH)
    canvas_y = int(CANVAS_HEIGHT - (robot_y * PIXELS_PER_INCH))
    return canvas_x, canvas_y

def constrain_servo_angle(angle, servo_num=1):
    """Ensure servo angle is within limits"""
    if servo_num == 1:
        return max(SERVO1_MIN, min(SERVO1_MAX, angle))
    else:
        return max(SERVO2_MIN, min(SERVO2_MAX, angle))

def apply_servo_offset(angle1, angle2):
    """Apply calibration offsets to servo angles"""
    angle1_adjusted = angle1 + SERVO1_OFFSET
    angle2_adjusted = angle2 + SERVO2_OFFSET
    
    angle1_final = constrain_servo_angle(angle1_adjusted, 1)
    angle2_final = constrain_servo_angle(angle2_adjusted, 2)
    
    return angle1_final, angle2_final
