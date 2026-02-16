"""
SCARA Robot Inverse Kinematics
Converts Cartesian (X, Y) coordinates to joint angles (theta1, theta2)
"""

import math
from config import ARM_L1, ARM_L2


class SCARAKinematics:
    """
    SCARA (Selective Compliance Assembly Robot Arm) inverse kinematics
    
    Configuration:
    - Two revolute joints in horizontal plane
    - L1: Length of first arm segment (shoulder to elbow)
    - L2: Length of second arm segment (elbow to wrist)
    """
    
    def __init__(self, L1=ARM_L1, L2=ARM_L2):
        self.L1 = L1
        self.L2 = L2
        self.max_reach = L1 + L2
        self.min_reach = abs(L1 - L2)
    
    def inverse_kinematics(self, x, y, elbow_up=True):
        """
        Calculate joint angles for target position (x, y)
        
        Args:
            x: Target X coordinate (inches)
            y: Target Y coordinate (inches)
            elbow_up: True for elbow-up configuration, False for elbow-down
        
        Returns:
            (theta1, theta2): Joint angles in degrees
        
        Raises:
            ValueError: If position is unreachable
        """
        # Calculate distance from origin to target
        r = math.sqrt(x**2 + y**2)
        
        # Check reachability
        if r > self.max_reach:
            raise ValueError(
                f"Position ({x:.2f}, {y:.2f}) out of reach: "
                f"distance {r:.2f}\" > max reach {self.max_reach:.2f}\""
            )
        
        if r < self.min_reach:
            raise ValueError(
                f"Position ({x:.2f}, {y:.2f}) too close: "
                f"distance {r:.2f}\" < min reach {self.min_reach:.2f}\""
            )
        
        # Special case: target at origin
        if r < 0.001:
            return 0.0, 0.0
        
        # Law of cosines to find elbow angle (theta2)
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        
        # Clamp to valid range to handle numerical errors
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        
        # Calculate theta2 (elbow angle)
        if elbow_up:
            theta2 = math.acos(cos_theta2)  # Positive angle (elbow up)
        else:
            theta2 = -math.acos(cos_theta2)  # Negative angle (elbow down)
        
        # Calculate shoulder angle (theta1)
        # Using atan2 to handle all quadrants correctly
        k1 = self.L1 + self.L2 * math.cos(theta2)
        k2 = self.L2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        
        # Convert to degrees
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        
        return theta1_deg, theta2_deg
    
    def forward_kinematics(self, theta1_deg, theta2_deg):
        """
        Calculate end-effector position from joint angles (for verification)
        
        Args:
            theta1_deg: Shoulder angle in degrees
            theta2_deg: Elbow angle in degrees
        
        Returns:
            (x, y): End-effector position in inches
        """
        theta1 = math.radians(theta1_deg)
        theta2 = math.radians(theta2_deg)
        
        # Elbow position
        elbow_x = self.L1 * math.cos(theta1)
        elbow_y = self.L1 * math.sin(theta1)
        
        # End-effector position
        x = elbow_x + self.L2 * math.cos(theta1 + theta2)
        y = elbow_y + self.L2 * math.sin(theta1 + theta2)
        
        return x, y
    
    def is_reachable(self, x, y):
        """
        Check if position (x, y) is within robot's workspace
        
        Args:
            x: Target X coordinate
            y: Target Y coordinate
        
        Returns:
            bool: True if reachable, False otherwise
        """
        r = math.sqrt(x**2 + y**2)
        return self.min_reach <= r <= self.max_reach
    
    def get_workspace_bounds(self):
        """
        Get workspace boundary information
        
        Returns:
            dict: Contains max_reach, min_reach, and configuration info
        """
        return {
            'max_reach': self.max_reach,
            'min_reach': self.min_reach,
            'L1': self.L1,
            'L2': self.L2,
            'config': 'SCARA 2-DOF'
        }


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def interpolate_arc(x_start, y_start, x_end, y_end, x_center, y_center, 
                    clockwise=True, segments_per_inch=10):
    """
    Generate waypoints along a circular arc
    
    Args:
        x_start, y_start: Start point
        x_end, y_end: End point
        x_center, y_center: Arc center point
        clockwise: True for G02 (CW), False for G03 (CCW)
        segments_per_inch: Resolution of arc interpolation
    
    Returns:
        list of (x, y) waypoints
    """
    # Calculate start and end angles
    start_angle = math.atan2(y_start - y_center, x_start - x_center)
    end_angle = math.atan2(y_end - y_center, x_end - x_center)
    
    # Calculate radius
    radius = math.sqrt((x_start - x_center)**2 + (y_start - y_center)**2)
    
    # Calculate sweep angle
    if clockwise:
        if end_angle > start_angle:
            sweep = end_angle - start_angle - 2 * math.pi
        else:
            sweep = end_angle - start_angle
    else:
        if end_angle < start_angle:
            sweep = end_angle - start_angle + 2 * math.pi
        else:
            sweep = end_angle - start_angle
    
    # Calculate arc length and number of segments
    arc_length = abs(radius * sweep)
    num_segments = max(4, int(arc_length * segments_per_inch))
    
    # Generate waypoints
    waypoints = []
    for i in range(num_segments + 1):
        t = i / num_segments
        angle = start_angle + t * sweep
        x = x_center + radius * math.cos(angle)
        y = y_center + radius * math.sin(angle)
        waypoints.append((x, y))
    
    return waypoints


def interpolate_line(x_start, y_start, x_end, y_end, segments_per_inch=10):
    """
    Generate waypoints along a straight line
    
    Args:
        x_start, y_start: Start point
        x_end, y_end: End point
        segments_per_inch: Resolution of line interpolation
    
    Returns:
        list of (x, y) waypoints
    """
    distance = math.sqrt((x_end - x_start)**2 + (y_end - y_start)**2)
    num_segments = max(2, int(distance * segments_per_inch))
    
    waypoints = []
    for i in range(num_segments + 1):
        t = i / num_segments
        x = x_start + t * (x_end - x_start)
        y = y_start + t * (y_end - y_start)
        waypoints.append((x, y))
    
    return waypoints


# ============================================================================
# TESTING
# ============================================================================

if __name__ == "__main__":
    # Test kinematics
    print("Testing SCARA Kinematics...")
    print(f"Arm configuration: L1={ARM_L1}\", L2={ARM_L2}\"")
    print(f"Max reach: {ARM_L1 + ARM_L2}\"")
    print(f"Min reach: {abs(ARM_L1 - ARM_L2)}\"")
    print()
    
    kinematics = SCARAKinematics()
    
    test_points = [
        (6.0, 0.0),   # Along X-axis
        (0.0, 6.0),   # Along Y-axis
        (4.0, 4.0),   # Diagonal
        (8.0, 8.0),   # Near max reach
        (0.5, 0.5),   # Near origin
    ]
    
    print("Test Points:")
    print("-" * 70)
    for x, y in test_points:
        try:
            theta1, theta2 = kinematics.inverse_kinematics(x, y)
            # Verify with forward kinematics
            x_check, y_check = kinematics.forward_kinematics(theta1, theta2)
            error = math.sqrt((x - x_check)**2 + (y - y_check)**2)
            
            print(f"Target: ({x:5.2f}, {y:5.2f}) → "
                  f"θ1={theta1:7.2f}° θ2={theta2:7.2f}° → "
                  f"Check: ({x_check:5.2f}, {y_check:5.2f}) "
                  f"Error: {error:.4f}\"")
        except ValueError as e:
            print(f"Target: ({x:5.2f}, {y:5.2f}) → ERROR: {e}")
    
    print()
    print("Arc Interpolation Test:")
    print("-" * 70)
    arc_points = interpolate_arc(6, 0, 0, 6, 0, 0, clockwise=False, segments_per_inch=5)
    print(f"Quarter circle: {len(arc_points)} waypoints")
    for i, (x, y) in enumerate(arc_points[:5]):
        print(f"  Point {i}: ({x:.3f}, {y:.3f})")
