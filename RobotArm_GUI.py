#i am a pusgfigiggijrigj

class GCodeParser:
    def __init__(self):
        self.absolute_mode = True  # G90 vs G91
        self.units = 'inches'      # G20 vs G21
        self.current_pos = {'x': 0, 'y': 0}
        self.feed_rate = 4.0       # inches/sec
    
    def parse_line(self, line):
        # Remove comments and whitespace
        line = line.split(';')[0].strip()
        
        # Extract command and parameters
        command = self.extract_command(line)
        params = self.extract_parameters(line)
        
        return {'command': command, 'params': params}
    
    def execute_command(self, cmd_dict):
        command = cmd_dict['command']
        params = cmd_dict['params']
        
        if command == 'G0':
            return self.rapid_move(params)
        elif command == 'G1':
            return self.linear_move(params)
        elif command == 'G90':
            self.absolute_mode = True
        elif command == 'G91':
            self.absolute_mode = False
        # ... handle other commands

class SCARAKinematics:
    def __init__(self, L1, L2):
        self.L1 = L1  # Length of first arm segment
        self.L2 = L2  # Length of second arm segment
    
    def inverse_kinematics(self, x, y):
        """
        Convert Cartesian (x,y) to joint angles (theta1, theta2)
        Returns angles in degrees
        """
        import math
        
        # Distance from origin to target
        r = math.sqrt(x**2 + y**2)
        
        # Check if position is reachable
        if r > (self.L1 + self.L2) or r < abs(self.L1 - self.L2):
            raise ValueError("Position out of reach")
        
        # Law of cosines for theta2
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        theta2 = math.acos(cos_theta2)
        
        # Calculate theta1
        k1 = self.L1 + self.L2 * math.cos(theta2)
        k2 = self.L2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        
        return math.degrees(theta1), math.degrees(theta2)
    
import numpy as np

class MotionController:
    def __init__(self, kinematics):
        self.kinematics = kinematics
        self.max_speed = 4.0  # inches/sec
        self.emergency_stop = False
    
    def generate_trajectory(self, start, end, speed):
        """
        Generate waypoints between start and end positions
        to achieve desired speed while meeting accuracy requirements
        """        
        # Calculate distance and time
        distance = np.sqrt((end['x']-start['x'])**2 + (end['y']-start['y'])**2)
        time_required = distance / speed
        
        # Generate waypoints (more points = better accuracy)
        num_points = max(10, int(distance * 10))  # 10 points per inch
        waypoints = []
        
        for i in range(num_points + 1):
            t = i / num_points
            x = start['x'] + t * (end['x'] - start['x'])
            y = start['y'] + t * (end['y'] - start['y'])
            waypoints.append({'x': x, 'y': y})
        
        return waypoints, time_required
    
    def emergency_stop_handler(self):
        """Stop all motion within 1 second"""
        self.emergency_stop = True
        # Implement deceleration profile here

    
import tkinter as tk
from tkinter import scrolledtext, filedialog

class RoboticArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("SCARA Arm Controller")
        
        # Initialize components
        self.parser = GCodeParser()
        self.kinematics = SCARAKinematics(L1=10, L2=10)  # Adjust lengths
        self.controller = MotionController(self.kinematics)
        
        self.setup_ui()
    
    def setup_ui(self):
        # G-code input area
        tk.Label(self.root, text="G-code Commands:").pack()
        self.gcode_text = scrolledtext.ScrolledText(self.root, height=10)
        self.gcode_text.pack(padx=10, pady=5)
        
        # Control buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)
        
        tk.Button(button_frame, text="Load File", 
                 command=self.load_file).grid(row=0, column=0, padx=5)
        tk.Button(button_frame, text="Execute", 
                 command=self.execute_gcode).grid(row=0, column=1, padx=5)
        tk.Button(button_frame, text="EMERGENCY STOP", 
                 bg="red", fg="white",
                 command=self.emergency_stop).grid(row=0, column=2, padx=5)
        
        # Status display
        tk.Label(self.root, text="Status:").pack()
        self.status_text = scrolledtext.ScrolledText(self.root, height=5)
        self.status_text.pack(padx=10, pady=5)
        
        # Position display
        self.position_label = tk.Label(self.root, 
                                      text="Current Position: X=0.00 Y=0.00")
        self.position_label.pack()
    
    def load_file(self):
        filename = filedialog.askopenfilename(
            filetypes=[("G-code files", "*.gcode *.nc"), ("All files", "*.*")]
        )
        if filename:
            with open(filename, 'r') as f:
                self.gcode_text.delete(1.0, tk.END)
                self.gcode_text.insert(1.0, f.read())
    
    def execute_gcode(self):
        gcode_lines = self.gcode_text.get(1.0, tk.END).split('\n')
        
        for line in gcode_lines:
            if self.controller.emergency_stop:
                self.status_text.insert(tk.END, "STOPPED BY EMERGENCY\n")
                break
            
            if line.strip():
                try:
                    parsed = self.parser.parse_line(line)
                    result = self.parser.execute_command(parsed)
                    
                    if result:  # Movement command
                        self.move_to_position(result)
                        
                except Exception as e:
                    self.status_text.insert(tk.END, f"Error: {str(e)}\n")
    
    def move_to_position(self, position):
        # Convert to joint angles
        theta1, theta2 = self.kinematics.inverse_kinematics(
            position['x'], position['y']
        )
        
        # Send to motors (you'll implement this based on your hardware)
        self.send_to_motors(theta1, theta2)
        
        # Update display
        self.position_label.config(
            text=f"Current Position: X={position['x']:.2f} Y={position['y']:.2f}"
        )
        self.root.update()
    
    def emergency_stop(self):
        self.controller.emergency_stop_handler()
        self.status_text.insert(tk.END, "EMERGENCY STOP ACTIVATED\n")
    
    def send_to_motors(self, theta1, theta2):
        # Interface with your motor controllers here
        # This could be serial communication, GPIO, etc.
        pass

# Run the application
if __name__ == "__main__":
    root = tk.Tk()
    app = RoboticArmGUI(root)
    root.mainloop()