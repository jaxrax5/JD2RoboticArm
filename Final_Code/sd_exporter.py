"""
SD Card Exporter
Converts G-code to Arduino-compatible format (angle1,angle2 per line)
"""

import os
from gcode_parser import GCodeParser
from kinematics import SCARAKinematics
from config import (ARM_L1, ARM_L2, SD_OUTPUT_FILE, SERVO1_HOME, SERVO2_HOME,
                   apply_servo_offset, HOME_X, HOME_Y)


class SDCardExporter:
    """
    Export G-code commands as servo angle pairs for Arduino SD card reader
    Format: "angle1,angle2" per line in moves.txt
    """
    
    def __init__(self):
        self.parser = GCodeParser()
        self.kinematics = SCARAKinematics(ARM_L1, ARM_L2)
        self.angle_list = []  # List of (angle1, angle2) tuples
        self.error_log = []
    
    def process_gcode(self, gcode_text):
        """
        Process G-code text and generate servo angles
        
        Args:
            gcode_text: String containing G-code commands (multi-line)
        
        Returns:
            tuple: (angle_list, error_log)
                - angle_list: List of (angle1, angle2) tuples
                - error_log: List of error messages
        """
        self.angle_list = []
        self.error_log = []
        
        # Add home position at start
        self.angle_list.append((SERVO1_HOME, SERVO2_HOME))
        
        lines = gcode_text.strip().split('\n')
        
        for line_num, line in enumerate(lines, 1):
            line = line.strip()
            
            # Skip empty lines and comments
            if not line or line.startswith(';'):
                continue
            
            try:
                # Parse line
                parsed = self.parser.parse_line(line)
                if parsed is None:
                    continue
                
                # Execute command
                result = self.parser.execute_command(parsed)
                if result is None:
                    continue
                
                # Process different command types
                cmd_type = result['command']
                
                if cmd_type in ['MOVE_LINEAR', 'MOVE_ARC', 'HOME']:
                    # These commands have waypoints - convert each to angles
                    waypoints = result.get('waypoints', [])
                    
                    for x, y in waypoints:
                        try:
                            theta1, theta2 = self.kinematics.inverse_kinematics(x, y)
                            
                            # Apply calibration offsets
                            theta1_adj, theta2_adj = apply_servo_offset(theta1, theta2)
                            
                            # Convert to integer servo angles
                            servo1 = int(round(theta1_adj))
                            servo2 = int(round(theta2_adj))
                            
                            self.angle_list.append((servo1, servo2))
                            
                        except ValueError as e:
                            error_msg = f"Line {line_num}: Position ({x:.2f}, {y:.2f}) - {str(e)}"
                            self.error_log.append(error_msg)
                
                elif cmd_type == 'DWELL':
                    # For dwell, repeat last position for duration
                    # This creates a pause in the Arduino execution
                    duration = result.get('duration', 0)
                    if duration > 0 and self.angle_list:
                        last_angles = self.angle_list[-1]
                        # Add duplicate positions (Arduino will stay at this position)
                        num_repeats = int(duration * 2)  # 2 repeats per second
                        for _ in range(num_repeats):
                            self.angle_list.append(last_angles)
                
                elif cmd_type == 'END':
                    # Program end - return to home
                    self.angle_list.append((SERVO1_HOME, SERVO2_HOME))
                    break
                
                # Other commands (MODE, UNITS, TOOL_CHANGE, SET_POSITION) 
                # don't generate movement, so no angles needed
                
            except Exception as e:
                error_msg = f"Line {line_num} '{line}': {str(e)}"
                self.error_log.append(error_msg)
        
        return self.angle_list, self.error_log
    
    def export_to_file(self, filename=SD_OUTPUT_FILE):
        """
        Export angle list to file for Arduino SD card
        
        Args:
            filename: Output filename (default: moves.txt)
        
        Returns:
            str: Path to output file or None if error
        """
        if not self.angle_list:
            self.error_log.append("No angles to export - process G-code first")
            return None
        
        try:
            with open(filename, 'w') as f:
                for angle1, angle2 in self.angle_list:
                    f.write(f"{angle1},{angle2}\n")
            
            return os.path.abspath(filename)
        
        except Exception as e:
            self.error_log.append(f"File write error: {str(e)}")
            return None
    
    def get_statistics(self):
        """
        Get statistics about the exported motion
        
        Returns:
            dict: Statistics including point count, range, etc.
        """
        if not self.angle_list:
            return {}
        
        angles1 = [a[0] for a in self.angle_list]
        angles2 = [a[1] for a in self.angle_list]
        
        return {
            'total_points': len(self.angle_list),
            'servo1_range': (min(angles1), max(angles1)),
            'servo2_range': (min(angles2), max(angles2)),
            'estimated_time_seconds': len(self.angle_list) * 0.1,  # Rough estimate
            'errors': len(self.error_log)
        }
    
    def preview_angles(self, max_lines=10):
        """
        Get preview of angle data
        
        Args:
            max_lines: Maximum number of lines to return
        
        Returns:
            str: Formatted preview text
        """
        if not self.angle_list:
            return "No data to preview"
        
        preview_lines = []
        preview_lines.append("Servo Angles Preview:")
        preview_lines.append("-" * 30)
        
        for i, (a1, a2) in enumerate(self.angle_list[:max_lines]):
            preview_lines.append(f"{i+1:4d}: {a1:3d}, {a2:3d}")
        
        if len(self.angle_list) > max_lines:
            preview_lines.append(f"... ({len(self.angle_list) - max_lines} more lines)")
        
        return "\n".join(preview_lines)


# ============================================================================
# CONVENIENCE FUNCTIONS
# ============================================================================

def gcode_to_sd_card(gcode_text, output_file=SD_OUTPUT_FILE):
    """
    One-step conversion: G-code text → SD card file
    
    Args:
        gcode_text: String containing G-code commands
        output_file: Output filename
    
    Returns:
        tuple: (success, filepath, errors)
    """
    exporter = SDCardExporter()
    
    # Process G-code
    angles, errors = exporter.process_gcode(gcode_text)
    
    if errors:
        return False, None, errors
    
    # Export to file
    filepath = exporter.export_to_file(output_file)
    
    if filepath:
        return True, filepath, []
    else:
        return False, None, exporter.error_log


# ============================================================================
# TESTING
# ============================================================================

if __name__ == "__main__":
    print("Testing SD Card Exporter...")
    print("=" * 70)
    
    # Sample G-code program - draw a square
    sample_gcode = """
G90           ; Absolute positioning
G20           ; Inches
G28           ; Home
G0 X4 Y4 F5   ; Move to start
G1 X6 Y4 F2   ; Draw line 1
G1 X6 Y6      ; Draw line 2
G1 X4 Y6      ; Draw line 3
G1 X4 Y4      ; Draw line 4 (close square)
G28           ; Return home
M2            ; End program
"""
    
    print("Sample G-code:")
    print(sample_gcode)
    print()
    
    # Create exporter and process
    exporter = SDCardExporter()
    angles, errors = exporter.process_gcode(sample_gcode)
    
    # Show results
    if errors:
        print("Errors encountered:")
        for error in errors:
            print(f"  - {error}")
        print()
    
    print(exporter.preview_angles(20))
    print()
    
    # Statistics
    stats = exporter.get_statistics()
    print("Statistics:")
    for key, value in stats.items():
        print(f"  {key}: {value}")
    print()
    
    # Export to file
    output_path = exporter.export_to_file('test_moves.txt')
    if output_path:
        print(f"✓ Exported to: {output_path}")
        
        # Show file contents
        print("\nFile contents (first 10 lines):")
        with open(output_path, 'r') as f:
            for i, line in enumerate(f):
                if i >= 10:
                    print("...")
                    break
                print(f"  {line.rstrip()}")
    else:
        print("✗ Export failed")
        for error in exporter.error_log:
            print(f"  {error}")
