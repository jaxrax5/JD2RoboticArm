"""
SCARA Robot Controller GUI - SD Card Mode
Main application with Canvas Drawing and G-code input modes
"""

import tkinter as tk
from tkinter import scrolledtext, filedialog, messagebox, ttk
import math

from config import *
from gcode_parser import GCodeParser
from kinematics import SCARAKinematics
from sd_exporter import SDCardExporter


class RobotControllerGUI:
    """Main application window"""
    
    def __init__(self, root):
        self.root = root
        self.root.title(WINDOW_TITLE)
        self.root.geometry("1100x850")
        
        # Backend components
        self.parser = GCodeParser()
        self.kinematics = SCARAKinematics(ARM_L1, ARM_L2)
        self.exporter = SDCardExporter()
        
        # Drawing mode variables
        self.drawing_enabled = False
        self.last_canvas_x = None
        self.last_canvas_y = None
        self.drawing_path = []  # Store (x, y) points for export
        
        # Setup UI
        self.setup_ui()
        
        # Show workspace info
        self.log_canvas(f"Workspace: {PAPER_WIDTH_INCHES}\" × {PAPER_HEIGHT_INCHES}\" paper")
        self.log_canvas(f"Robot reach: {self.kinematics.max_reach:.1f}\" max")
        self.log_gcode(f"Ready. Robot configuration: L1={ARM_L1}\", L2={ARM_L2}\"")
    
    def setup_ui(self):
        """Create the user interface"""
        
        # ===== Top Control Panel =====
        top_frame = tk.Frame(self.root, bg="#2c3e50", height=60)
        top_frame.pack(fill=tk.X, side=tk.TOP)
        top_frame.pack_propagate(False)
        
        # Mode selection
        tk.Label(top_frame, text="Mode:", font=("Arial", 12, "bold"), 
                bg="#2c3e50", fg="white").pack(side=tk.LEFT, padx=15)
        
        self.mode_var = tk.StringVar(value="canvas")
        tk.Radiobutton(top_frame, text="Canvas Drawing", variable=self.mode_var,
                      value="canvas", command=self.switch_mode, font=("Arial", 11),
                      bg="#2c3e50", fg="white", selectcolor="#34495e",
                      activebackground="#34495e", activeforeground="white").pack(side=tk.LEFT, padx=5)
        tk.Radiobutton(top_frame, text="G-code Commands", variable=self.mode_var,
                      value="gcode", command=self.switch_mode, font=("Arial", 11),
                      bg="#2c3e50", fg="white", selectcolor="#34495e",
                      activebackground="#34495e", activeforeground="white").pack(side=tk.LEFT, padx=5)
        
        # Emergency stop button
        tk.Button(top_frame, text="⚠ EMERGENCY STOP", bg="#e74c3c", fg="white",
                 font=("Arial", 12, "bold"), command=self.emergency_stop,
                 relief=tk.RAISED, borderwidth=3).pack(side=tk.RIGHT, padx=15, pady=10)
        
        # ===== Main Content Area (Notebook) =====
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Canvas Drawing Tab
        self.canvas_frame = tk.Frame(self.notebook)
        self.notebook.add(self.canvas_frame, text="📐 Canvas Drawing")
        self.setup_canvas_tab()
        
        # G-code Tab
        self.gcode_frame = tk.Frame(self.notebook)
        self.notebook.add(self.gcode_frame, text="📝 G-code Commands")
        self.setup_gcode_tab()
        
        # ===== Bottom Status Bar =====
        status_frame = tk.Frame(self.root, relief=tk.SUNKEN, borderwidth=2, bg="#ecf0f1")
        status_frame.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=5)
        
        self.position_label = tk.Label(status_frame, 
                                       text="Position: X=0.00\" Y=0.00\" | θ1=0° θ2=0°",
                                       font=("Courier", 10), bg="#ecf0f1")
        self.position_label.pack(side=tk.LEFT, padx=10)
        
        self.status_label = tk.Label(status_frame, text="Ready", 
                                     font=("Arial", 10), bg="#ecf0f1")
        self.status_label.pack(side=tk.RIGHT, padx=10)
        
        # Start in canvas mode
        self.switch_mode()
    
    def setup_canvas_tab(self):
        """Setup canvas drawing interface"""
        
        # Top info panel
        info_frame = tk.Frame(self.canvas_frame, bg="#3498db", height=50)
        info_frame.pack(fill=tk.X, pady=(0, 5))
        info_frame.pack_propagate(False)
        
        tk.Label(info_frame, 
                text=f"Draw freehand on the canvas (representing {PAPER_WIDTH_INCHES}\" × {PAPER_HEIGHT_INCHES}\" paper)",
                font=("Arial", 11), bg="#3498db", fg="white").pack(pady=12)
        
        # Control panel
        control_frame = tk.Frame(self.canvas_frame)
        control_frame.pack(pady=10)
        
        self.draw_button = tk.Button(control_frame, text="Enable Drawing",
                                     command=self.toggle_drawing,
                                     bg="#2ecc71", fg="white", font=("Arial", 10, "bold"),
                                     width=15, height=2)
        self.draw_button.grid(row=0, column=0, padx=5)
        
        tk.Button(control_frame, text="Clear Canvas",
                 command=self.clear_canvas,
                 bg="#95a5a6", fg="white", font=("Arial", 10),
                 width=15, height=2).grid(row=0, column=1, padx=5)
        
        tk.Button(control_frame, text="Export to SD Card",
                 command=self.export_canvas_to_sd,
                 bg="#e67e22", fg="white", font=("Arial", 10, "bold"),
                 width=15, height=2).grid(row=0, column=2, padx=5)
        
        # Canvas container
        canvas_container = tk.Frame(self.canvas_frame, relief=tk.SUNKEN, borderwidth=3)
        canvas_container.pack(padx=20, pady=10)
        
        # Draw workspace boundaries
        self.canvas = tk.Canvas(canvas_container, bg="white",
                               width=CANVAS_WIDTH, height=CANVAS_HEIGHT,
                               cursor="crosshair")
        self.canvas.pack()
        
        # Draw workspace circle (reachable area)
        self.draw_workspace_boundary()
        
        # Bind mouse events
        self.canvas.bind("<Button-1>", self.start_draw)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.end_draw)
        
        # Canvas log
        log_frame = tk.Frame(self.canvas_frame)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        
        tk.Label(log_frame, text="Drawing Log:", font=("Arial", 10, "bold")).pack(anchor=tk.W)
        self.canvas_log = scrolledtext.ScrolledText(log_frame, height=8, font=("Courier", 9))
        self.canvas_log.pack(fill=tk.BOTH, expand=True)
    
    def setup_gcode_tab(self):
        """Setup G-code command interface"""
        
        # Top info panel
        info_frame = tk.Frame(self.gcode_frame, bg="#9b59b6", height=50)
        info_frame.pack(fill=tk.X, pady=(0, 5))
        info_frame.pack_propagate(False)
        
        tk.Label(info_frame,
                text="Enter G-code commands to generate servo angles for SD card",
                font=("Arial", 11), bg="#9b59b6", fg="white").pack(pady=12)
        
        # Main layout: Left (input) and Right (output)
        main_layout = tk.Frame(self.gcode_frame)
        main_layout.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # ===== LEFT SIDE: G-code Input =====
        left_frame = tk.Frame(main_layout)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        tk.Label(left_frame, text="G-code Commands:", 
                font=("Arial", 10, "bold")).pack(anchor=tk.W)
        
        # G-code text area
        self.gcode_text = scrolledtext.ScrolledText(left_frame, height=20, 
                                                    font=("Courier", 10))
        self.gcode_text.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Load sample G-code
        sample_gcode = self.get_sample_gcode()
        self.gcode_text.insert(1.0, sample_gcode)
        
        # Control buttons
        button_frame = tk.Frame(left_frame)
        button_frame.pack(pady=10)
        
        tk.Button(button_frame, text="📁 Load File",
                 command=self.load_gcode_file, width=12).grid(row=0, column=0, padx=3)
        tk.Button(button_frame, text="💾 Save G-code",
                 command=self.save_gcode_file, width=12).grid(row=0, column=1, padx=3)
        tk.Button(button_frame, text="🔄 Process",
                 command=self.process_gcode, bg="#3498db", fg="white",
                 font=("Arial", 10, "bold"), width=12).grid(row=0, column=2, padx=3)
        tk.Button(button_frame, text="📤 Export to SD",
                 command=self.export_gcode_to_sd, bg="#e67e22", fg="white",
                 font=("Arial", 10, "bold"), width=12).grid(row=0, column=3, padx=3)
        
        # ===== RIGHT SIDE: Output Preview =====
        right_frame = tk.Frame(main_layout)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        tk.Label(right_frame, text="Servo Angles Preview (moves.txt):",
                font=("Arial", 10, "bold")).pack(anchor=tk.W)
        
        self.angle_preview = scrolledtext.ScrolledText(right_frame, height=20,
                                                       font=("Courier", 9),
                                                       state=tk.DISABLED)
        self.angle_preview.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Statistics display
        stats_frame = tk.Frame(right_frame, relief=tk.GROOVE, borderwidth=2)
        stats_frame.pack(fill=tk.X, pady=5)
        
        self.stats_label = tk.Label(stats_frame, text="Statistics: Not processed yet",
                                    font=("Courier", 9), justify=tk.LEFT, anchor=tk.W)
        self.stats_label.pack(padx=5, pady=5)
        
        # ===== BOTTOM: Execution Log =====
        tk.Label(self.gcode_frame, text="Execution Log:",
                font=("Arial", 10, "bold")).pack(anchor=tk.W, padx=10)
        
        self.gcode_log = scrolledtext.ScrolledText(self.gcode_frame, height=6,
                                                   font=("Courier", 9))
        self.gcode_log.pack(fill=tk.X, padx=10, pady=5)
    
    # =========================================================================
    # MODE SWITCHING
    # =========================================================================
    
    def switch_mode(self):
        """Switch between canvas and G-code modes"""
        mode = self.mode_var.get()
        if mode == "canvas":
            self.notebook.select(self.canvas_frame)
            self.update_status("Canvas Drawing Mode")
        else:
            self.notebook.select(self.gcode_frame)
            self.update_status("G-code Command Mode")
    
    # =========================================================================
    # CANVAS DRAWING MODE
    # =========================================================================
    
    def draw_workspace_boundary(self):
        """Draw robot workspace boundary on canvas"""
        # Calculate center of paper
        center_x = CANVAS_WIDTH // 2
        center_y = CANVAS_HEIGHT // 2
        
        # Draw max reach circle
        max_reach_pixels = self.kinematics.max_reach * PIXELS_PER_INCH
        self.canvas.create_oval(
            center_x - max_reach_pixels, center_y - max_reach_pixels,
            center_x + max_reach_pixels, center_y + max_reach_pixels,
            outline="#2ecc71", width=2, dash=(5, 5)
        )
        
        # Draw min reach circle (dead zone)
        if self.kinematics.min_reach > 0:
            min_reach_pixels = self.kinematics.min_reach * PIXELS_PER_INCH
            self.canvas.create_oval(
                center_x - min_reach_pixels, center_y - min_reach_pixels,
                center_x + min_reach_pixels, center_y + min_reach_pixels,
                outline="#e74c3c", width=2, dash=(5, 5)
            )
        
        # Draw center point
        self.canvas.create_oval(center_x-3, center_y-3, center_x+3, center_y+3,
                               fill="#3498db", outline="")
    
    def toggle_drawing(self):
        """Enable/disable drawing on canvas"""
        self.drawing_enabled = not self.drawing_enabled
        if self.drawing_enabled:
            self.draw_button.config(text="Disable Drawing", bg="#e74c3c")
            self.log_canvas("✓ Drawing enabled - drag mouse to draw")
            self.drawing_path = []  # Clear path
        else:
            self.draw_button.config(text="Enable Drawing", bg="#2ecc71")
            self.log_canvas("✗ Drawing disabled")
    
    def start_draw(self, event):
        """Start drawing stroke"""
        if not self.drawing_enabled:
            return
        self.last_canvas_x = event.x
        self.last_canvas_y = event.y
    
    def draw(self, event):
        """Draw line and track position"""
        if not self.drawing_enabled or self.last_canvas_x is None:
            return
        
        # Draw line on canvas
        self.canvas.create_line(
            self.last_canvas_x, self.last_canvas_y, event.x, event.y,
            width=BRUSH_SIZE, fill=DRAW_COLOR, capstyle=tk.ROUND,
            smooth=True
        )
        
        # Convert to robot coordinates
        robot_x, robot_y = canvas_to_robot(event.x, event.y)
        
        # Store point
        self.drawing_path.append((robot_x, robot_y))
        
        # Try to calculate angles
        try:
            theta1, theta2 = self.kinematics.inverse_kinematics(robot_x, robot_y)
            self.update_position_display(robot_x, robot_y, theta1, theta2)
        except ValueError:
            # Out of reach - don't update display
            pass
        
        self.last_canvas_x = event.x
        self.last_canvas_y = event.y
    
    def end_draw(self, event):
        """End drawing stroke"""
        self.last_canvas_x = None
        self.last_canvas_y = None
        if self.drawing_path:
            self.log_canvas(f"Path captured: {len(self.drawing_path)} points")
    
    def clear_canvas(self):
        """Clear all drawings"""
        self.canvas.delete("all")
        self.draw_workspace_boundary()
        self.drawing_path = []
        self.log_canvas("Canvas cleared")
    
    def export_canvas_to_sd(self):
        """Export canvas drawing as G-code then to SD card format"""
        if not self.drawing_path:
            messagebox.showwarning("No Drawing", "Please draw something first!")
            return
        
        self.log_canvas(f"Exporting {len(self.drawing_path)} points...")
        
        # Convert path to G-code
        gcode_lines = ["G90", "G20", "G28"]  # Setup
        
        # First point - rapid move
        x0, y0 = self.drawing_path[0]
        gcode_lines.append(f"G0 X{x0:.3f} Y{y0:.3f}")
        
        # Rest of points - linear moves
        for x, y in self.drawing_path[1:]:
            gcode_lines.append(f"G1 X{x:.3f} Y{y:.3f} F{DEFAULT_FEED_RATE}")
        
        gcode_lines.append("G28")  # Home
        gcode_lines.append("M2")   # End
        
        gcode_text = "\n".join(gcode_lines)
        
        # Export using SD exporter
        success, filepath, errors = self.exporter_process_and_save(gcode_text)
        
        if success:
            self.log_canvas(f"✓ Exported to: {filepath}")
            messagebox.showinfo("Export Successful",
                              f"Drawing exported to SD card file:\n{filepath}\n\n"
                              f"Copy this file to your SD card as 'program.gcode'")
        else:
            self.log_canvas("✗ Export failed")
            for error in errors:
                self.log_canvas(f"  {error}")
            messagebox.showerror("Export Failed", "\n".join(errors))
    
    # =========================================================================
    # G-CODE MODE
    # =========================================================================
    
    def get_sample_gcode(self):
        """Return sample G-code program"""
        return """G90           ; Absolute positioning
G20           ; Units in inches
G28           ; Auto-home
G0 X4 Y4 F5   ; Rapid move to start
G1 X6 Y4 F2   ; Draw horizontal line
G1 X6 Y6      ; Draw vertical line
G1 X4 Y6      ; Draw horizontal line
G1 X4 Y4      ; Close the square
G4 P1         ; Pause 1 second
G28           ; Return home
M2            ; Program end"""
    
    def load_gcode_file(self):
        """Load G-code from file"""
        filename = filedialog.askopenfilename(
            title="Select G-code file",
            filetypes=[("G-code files", "*.gcode *.nc *.txt"), ("All files", "*.*")]
        )
        if filename:
            try:
                with open(filename, 'r') as f:
                    self.gcode_text.delete(1.0, tk.END)
                    self.gcode_text.insert(1.0, f.read())
                self.log_gcode(f"✓ Loaded: {filename}")
            except Exception as e:
                self.log_gcode(f"✗ Error loading file: {str(e)}")
                messagebox.showerror("Load Error", str(e))
    
    def save_gcode_file(self):
        """Save G-code to file"""
        filename = filedialog.asksaveasfilename(
            title="Save G-code file",
            defaultextension=".gcode",
            filetypes=[("G-code files", "*.gcode"), ("Text files", "*.txt"), ("All files", "*.*")]
        )
        if filename:
            try:
                with open(filename, 'w') as f:
                    f.write(self.gcode_text.get(1.0, tk.END))
                self.log_gcode(f"✓ Saved: {filename}")
            except Exception as e:
                self.log_gcode(f"✗ Error saving file: {str(e)}")
                messagebox.showerror("Save Error", str(e))
    
    def process_gcode(self):
        """Process G-code and show preview"""
        gcode_text = self.gcode_text.get(1.0, tk.END)
        
        self.log_gcode("=" * 50)
        self.log_gcode("Processing G-code...")
        
        # Process with exporter
        angles, errors = self.exporter.process_gcode(gcode_text)
        
        # Show errors
        if errors:
            self.log_gcode(f"⚠ {len(errors)} errors encountered:")
            for error in errors[:10]:  # Show first 10
                self.log_gcode(f"  {error}")
            if len(errors) > 10:
                self.log_gcode(f"  ... and {len(errors)-10} more errors")
        else:
            self.log_gcode("✓ No errors")
        
        # Update preview
        self.angle_preview.config(state=tk.NORMAL)
        self.angle_preview.delete(1.0, tk.END)
        
        preview_text = self.exporter.preview_angles(50)
        self.angle_preview.insert(1.0, preview_text)
        self.angle_preview.config(state=tk.DISABLED)
        
        # Update statistics
        stats = self.exporter.get_statistics()
        stats_text = (
            f"Total Points: {stats.get('total_points', 0)}\n"
            f"Servo 1 Range: {stats.get('servo1_range', (0, 0))}\n"
            f"Servo 2 Range: {stats.get('servo2_range', (0, 0))}\n"
            f"Est. Time: {stats.get('estimated_time_seconds', 0):.1f} seconds\n"
            f"Errors: {stats.get('errors', 0)}"
        )
        self.stats_label.config(text=stats_text)
        
        self.log_gcode("✓ Processing complete")
        self.log_gcode("=" * 50)
    
    def export_gcode_to_sd(self):
        """Export processed G-code to SD card file"""
        gcode_text = self.gcode_text.get(1.0, tk.END)
        
        success, filepath, errors = self.exporter_process_and_save(gcode_text)
        
        if success:
            self.log_gcode(f"✓ Exported to: {filepath}")
            messagebox.showinfo("Export Successful",
                              f"G-code exported to SD card file:\n{filepath}\n\n"
                             f"Copy this file to your SD card as 'program.gcode'\n"
                              f"Then insert the SD card into your robot controller.")
        else:
            self.log_gcode("✗ Export failed")
            for error in errors:
                self.log_gcode(f"  {error}")
            messagebox.showerror("Export Failed", "\n".join(errors))
    
    def exporter_process_and_save(self, gcode_text):
        """
        Save raw G-code directly as program.gcode
        This matches the Arduino firmware expectation.
        """

        # Ask where to save
        filename = filedialog.asksaveasfilename(
            title="Save G-code for SD Card",
            initialfile="program.gcode",
            defaultextension=".gcode",
            filetypes=[("G-code files", "*.gcode"), ("All files", "*.*")]
        )

        if not filename:
            return False, None, ["Save cancelled by user"]

        try:
            with open(filename, "w") as f:
                f.write(gcode_text.strip() + "\n")

            return True, filename, []

        except Exception as e:
            return False, None, [str(e)]

    
    # =========================================================================
    # UTILITY METHODS
    # =========================================================================
    
    def update_position_display(self, x, y, theta1, theta2):
        """Update position display in status bar"""
        self.position_label.config(
            text=f"Position: X={x:.2f}\" Y={y:.2f}\" | θ1={theta1:.1f}° θ2={theta2:.1f}°"
        )
    
    def update_status(self, message):
        """Update status label"""
        self.status_label.config(text=message)
    
    def log_canvas(self, message):
        """Add message to canvas log"""
        self.canvas_log.insert(tk.END, f"{message}\n")
        self.canvas_log.see(tk.END)
    
    def log_gcode(self, message):
        """Add message to G-code log"""
        self.gcode_log.insert(tk.END, f"{message}\n")
        self.gcode_log.see(tk.END)
    
    def emergency_stop(self):
        """Emergency stop - disable all drawing"""
        self.drawing_enabled = False
        self.draw_button.config(text="Enable Drawing", bg="#2ecc71")
        self.update_status("⚠ EMERGENCY STOP")
        self.log_canvas("⚠ EMERGENCY STOP ACTIVATED")
        self.log_gcode("⚠ EMERGENCY STOP ACTIVATED")
        messagebox.showwarning("Emergency Stop", 
                              "All operations halted.\nRe-enable drawing when ready.")


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControllerGUI(root)
    root.mainloop()
