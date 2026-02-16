"""
Enhanced G-code Parser
Supports: G0, G1, G2, G3, G4, G20, G21, G28, G90, G91, G92, M2, M6
"""

import time
from config import (DEFAULT_FEED_RATE, MAX_FEED_RATE, HOME_X, HOME_Y, 
                   DEFAULT_UNITS, MM_PER_INCH, ARC_SEGMENTS_PER_INCH)
from kinematics import interpolate_arc, interpolate_line


class GCodeParser:
    """
    Parse and execute G-code commands for SCARA robot
    """
    
    def __init__(self):
        # Position mode
        self.absolute_mode = True  # G90 (absolute) vs G91 (incremental)
        
        # Units
        self.units = DEFAULT_UNITS  # 'inches' or 'mm'
        
        # Current position
        self.current_pos = {'x': HOME_X, 'y': HOME_Y}
        
        # Coordinate system offset (G92)
        self.offset = {'x': 0.0, 'y': 0.0}
        
        # Feed rate
        self.feed_rate = DEFAULT_FEED_RATE  # inches/sec or mm/sec
        
        # Home position
        self.home_pos = {'x': HOME_X, 'y': HOME_Y}
        
        # Command history
        self.command_count = 0
    
    def parse_line(self, line):
        """
        Parse a single line of G-code
        
        Args:
            line: String containing G-code command
        
        Returns:
            dict with 'command', 'params', 'raw' or None if empty/comment
        """
        # Remove comments (everything after ';')
        line = line.split(';')[0].strip()
        
        # Skip empty lines
        if not line:
            return None
        
        # Split into parts
        parts = line.upper().split()
        if not parts:
            return None
        
        # First part is the command
        command = parts[0]
        
        # Extract parameters (X, Y, Z, I, J, F, P, etc.)
        params = {}
        for part in parts[1:]:
            if len(part) >= 2:
                key = part[0]
                try:
                    value = float(part[1:])
                    params[key] = value
                except ValueError:
                    pass  # Ignore invalid parameters
        
        return {
            'command': command,
            'params': params,
            'raw': line
        }
    
    def execute_command(self, cmd_dict):
        """
        Execute a parsed G-code command
        
        Args:
            cmd_dict: Dictionary from parse_line()
        
        Returns:
            dict with command execution results or None
        """
        if cmd_dict is None:
            return None
        
        command = cmd_dict['command']
        params = cmd_dict['params']
        
        self.command_count += 1
        
        # ====== MOVEMENT COMMANDS ======
        
        if command == 'G0':  # Rapid positioning
            return self._process_linear_move(params, rapid=True)
        
        elif command == 'G1':  # Linear interpolation
            return self._process_linear_move(params, rapid=False)
        
        elif command == 'G2':  # Clockwise arc
            return self._process_arc_move(params, clockwise=True)
        
        elif command == 'G3':  # Counter-clockwise arc
            return self._process_arc_move(params, clockwise=False)
        
        elif command == 'G4':  # Dwell (pause)
            return self._process_dwell(params)
        
        # ====== COORDINATE SYSTEM COMMANDS ======
        
        elif command == 'G20':  # Inches
            self.units = 'inches'
            return {'command': 'UNITS', 'units': 'inches'}
        
        elif command == 'G21':  # Millimeters
            self.units = 'mm'
            return {'command': 'UNITS', 'units': 'mm'}
        
        elif command == 'G28':  # Auto-home
            return self._process_home(params)
        
        elif command == 'G90':  # Absolute positioning
            self.absolute_mode = True
            return {'command': 'MODE', 'mode': 'absolute'}
        
        elif command == 'G91':  # Incremental positioning
            self.absolute_mode = False
            return {'command': 'MODE', 'mode': 'incremental'}
        
        elif command == 'G92':  # Set position / coordinate system offset
            return self._process_set_position(params)
        
        # ====== MACHINE COMMANDS ======
        
        elif command == 'M2':  # Program end
            return {'command': 'END'}
        
        elif command == 'M6':  # Tool change
            tool_number = int(params.get('T', 0))
            return {'command': 'TOOL_CHANGE', 'tool': tool_number}
        
        # ====== UNKNOWN COMMAND ======
        else:
            return {'command': 'UNKNOWN', 'raw': cmd_dict['raw']}
    
    # ========================================================================
    # PRIVATE METHODS - Command Processors
    # ========================================================================
    
    def _process_linear_move(self, params, rapid=False):
        """Process G0 (rapid) or G1 (feed) linear move"""
        # Get target position
        if self.absolute_mode:
            # Absolute coordinates
            target_x = params.get('X', self.current_pos['x'])
            target_y = params.get('Y', self.current_pos['y'])
        else:
            # Incremental coordinates
            target_x = self.current_pos['x'] + params.get('X', 0)
            target_y = self.current_pos['y'] + params.get('Y', 0)
        
        # Apply coordinate system offset (G92)
        target_x -= self.offset['x']
        target_y -= self.offset['y']
        
        # Update feed rate if specified
        if 'F' in params:
            self.feed_rate = min(params['F'], MAX_FEED_RATE)
        
        # Use max speed for rapid moves
        speed = MAX_FEED_RATE if rapid else self.feed_rate
        
        # Generate waypoints for smooth motion
        waypoints = interpolate_line(
            self.current_pos['x'], self.current_pos['y'],
            target_x, target_y,
            segments_per_inch=5  # Lower resolution for efficiency
        )
        
        # Update current position
        self.current_pos = {'x': target_x, 'y': target_y}
        
        return {
            'command': 'MOVE_LINEAR',
            'start': waypoints[0],
            'end': waypoints[-1],
            'waypoints': waypoints,
            'speed': speed,
            'rapid': rapid
        }
    
    def _process_arc_move(self, params, clockwise):
        """Process G2 (CW) or G3 (CCW) arc move"""
        # Get target position (end of arc)
        if self.absolute_mode:
            target_x = params.get('X', self.current_pos['x'])
            target_y = params.get('Y', self.current_pos['y'])
        else:
            target_x = self.current_pos['x'] + params.get('X', 0)
            target_y = self.current_pos['y'] + params.get('Y', 0)
        
        # Apply offset
        target_x -= self.offset['x']
        target_y -= self.offset['y']
        
        # Get arc center offset (I, J are offsets from start point)
        I = params.get('I', 0)  # X offset to center
        J = params.get('J', 0)  # Y offset to center
        
        # Calculate arc center (absolute coordinates)
        center_x = self.current_pos['x'] + I
        center_y = self.current_pos['y'] + J
        
        # Update feed rate if specified
        if 'F' in params:
            self.feed_rate = min(params['F'], MAX_FEED_RATE)
        
        # Generate arc waypoints
        waypoints = interpolate_arc(
            self.current_pos['x'], self.current_pos['y'],
            target_x, target_y,
            center_x, center_y,
            clockwise=clockwise,
            segments_per_inch=ARC_SEGMENTS_PER_INCH
        )
        
        # Update current position
        self.current_pos = {'x': target_x, 'y': target_y}
        
        return {
            'command': 'MOVE_ARC',
            'start': waypoints[0],
            'end': waypoints[-1],
            'center': (center_x, center_y),
            'waypoints': waypoints,
            'speed': self.feed_rate,
            'clockwise': clockwise
        }
    
    def _process_dwell(self, params):
        """Process G4 dwell (pause) command"""
        # P parameter: dwell time in seconds (or milliseconds depending on machine)
        # Standard is seconds for most controllers
        dwell_time = params.get('P', 0)
        
        return {
            'command': 'DWELL',
            'duration': dwell_time
        }
    
    def _process_home(self, params):
        """Process G28 auto-home command"""
        # G28 moves to pre-defined home position
        # Optional parameters X, Y specify intermediate point to pass through
        
        if 'X' in params or 'Y' in params:
            # Move to intermediate point first
            intermediate_x = params.get('X', self.current_pos['x'])
            intermediate_y = params.get('Y', self.current_pos['y'])
            
            return {
                'command': 'HOME',
                'intermediate': (intermediate_x, intermediate_y),
                'home': (self.home_pos['x'], self.home_pos['y'])
            }
        else:
            # Direct home
            target_x = self.home_pos['x']
            target_y = self.home_pos['y']
            
            waypoints = interpolate_line(
                self.current_pos['x'], self.current_pos['y'],
                target_x, target_y,
                segments_per_inch=5
            )
            
            self.current_pos = {'x': target_x, 'y': target_y}
            
            return {
                'command': 'HOME',
                'waypoints': waypoints,
                'home': (target_x, target_y)
            }
    
    def _process_set_position(self, params):
        """Process G92 set position command"""
        # G92 sets the coordinate system offset
        # Without parameters, it clears the offset
        # With parameters, it sets current position to specified values
        
        if not params:
            # Clear offset
            self.offset = {'x': 0.0, 'y': 0.0}
            return {
                'command': 'SET_POSITION',
                'action': 'clear_offset'
            }
        else:
            # Set coordinate system so that current position = specified values
            new_x = params.get('X', self.current_pos['x'])
            new_y = params.get('Y', self.current_pos['y'])
            
            # Calculate required offset
            self.offset['x'] = self.current_pos['x'] - new_x
            self.offset['y'] = self.current_pos['y'] - new_y
            
            return {
                'command': 'SET_POSITION',
                'action': 'set_offset',
                'new_position': (new_x, new_y),
                'offset': (self.offset['x'], self.offset['y'])
            }
    
    # ========================================================================
    # UTILITY METHODS
    # ========================================================================
    
    def get_current_position(self):
        """Get current position (after offset)"""
        return {
            'x': self.current_pos['x'] - self.offset['x'],
            'y': self.current_pos['y'] - self.offset['y']
        }
    
    def get_state(self):
        """Get parser state"""
        return {
            'position': self.get_current_position(),
            'absolute_mode': self.absolute_mode,
            'units': self.units,
            'feed_rate': self.feed_rate,
            'offset': self.offset.copy(),
            'command_count': self.command_count
        }
    
    def reset(self):
        """Reset parser to initial state"""
        self.__init__()


# ============================================================================
# TESTING
# ============================================================================

if __name__ == "__main__":
    print("Testing G-code Parser...")
    print("=" * 70)
    
    parser = GCodeParser()
    
    # Test commands
    test_commands = [
        "G90 ; Absolute mode",
        "G20 ; Inches",
        "G0 X4 Y4 ; Rapid to (4,4)",
        "G1 X6 Y4 F2.5 ; Linear move",
        "G2 X6 Y6 I1 J0 ; Clockwise arc",
        "G3 X4 Y6 I0 J1 ; Counter-clockwise arc",
        "G4 P1.5 ; Dwell 1.5 seconds",
        "G92 X0 Y0 ; Set position to (0,0)",
        "G1 X2 Y2 ; Move in offset coordinates",
        "G28 ; Auto-home",
        "M2 ; Program end"
    ]
    
    for cmd_line in test_commands:
        print(f"\nCommand: {cmd_line}")
        parsed = parser.parse_line(cmd_line)
        result = parser.execute_command(parsed)
        
        if result:
            print(f"  Result: {result['command']}")
            if 'waypoints' in result:
                print(f"  Waypoints: {len(result['waypoints'])} points")
                print(f"  Start: ({result['waypoints'][0][0]:.2f}, {result['waypoints'][0][1]:.2f})")
                print(f"  End: ({result['waypoints'][-1][0]:.2f}, {result['waypoints'][-1][1]:.2f})")
            if 'duration' in result:
                print(f"  Duration: {result['duration']}s")
            if 'offset' in result:
                print(f"  Offset: {result['offset']}")
    
    print("\n" + "=" * 70)
    print("Final parser state:")
    state = parser.get_state()
    for key, value in state.items():
        print(f"  {key}: {value}")
