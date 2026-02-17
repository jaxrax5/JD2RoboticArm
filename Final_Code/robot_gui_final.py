"""
SCARA Robot Controller GUI
Saves G-code directly to moves.txt for Arduino to parse.
Self-contained: only needs tkinter and math.

Supported commands written to moves.txt:
  G0  - Rapid move
  G1  - Linear move
  G2  - Clockwise arc
  G3  - Counter-clockwise arc
  G4  - Dwell / pause
  G20 - Inches
  G21 - Millimeters
  G28 - Home
  G90 - Absolute mode
  G91 - Incremental mode
  G92 - Set position
  M2  - Program end
  M6  - Tool change
"""

import tkinter as tk
from tkinter import scrolledtext, filedialog, messagebox, ttk
import math

# ============================================================================
# CONFIGURATION  —  edit to match your robot
# ============================================================================

ARM_L1              = 6.0   # inches — shoulder to elbow
ARM_L2              = 6.0   # inches — elbow to pen

PAPER_WIDTH_INCHES  = 8.5
PAPER_HEIGHT_INCHES = 11.0
PIXELS_PER_INCH     = 50    # canvas scale: 50 px = 1 inch

CANVAS_WIDTH  = int(PAPER_WIDTH_INCHES  * PIXELS_PER_INCH)   # 425 px
CANVAS_HEIGHT = int(PAPER_HEIGHT_INCHES * PIXELS_PER_INCH)   # 550 px

DEFAULT_FEED_RATE = 2.0   # inches/sec used when exporting canvas drawing
BRUSH_SIZE        = 3
DRAW_COLOR        = "black"
WINDOW_TITLE      = "SCARA Robot Controller"

# ============================================================================
# HELPERS
# ============================================================================

def canvas_to_robot(cx, cy):
    """Canvas pixels  →  robot workspace inches (flips Y axis)."""
    return cx / PIXELS_PER_INCH, (CANVAS_HEIGHT - cy) / PIXELS_PER_INCH


def inverse_kinematics(x, y):
    """
    Return (theta1_deg, theta2_deg) for Cartesian target (x, y) in inches.
    Raises ValueError when position is out of reach.
    """
    r = math.sqrt(x ** 2 + y ** 2)
    if r > (ARM_L1 + ARM_L2):
        raise ValueError(f"Too far: {r:.2f}\" > {ARM_L1+ARM_L2}\"")
    if r < abs(ARM_L1 - ARM_L2):
        raise ValueError(f"Too close: {r:.2f}\" < {abs(ARM_L1-ARM_L2)}\"")

    cos_t2 = (x**2 + y**2 - ARM_L1**2 - ARM_L2**2) / (2 * ARM_L1 * ARM_L2)
    cos_t2 = max(-1.0, min(1.0, cos_t2))
    t2     = math.acos(cos_t2)
    k1     = ARM_L1 + ARM_L2 * math.cos(t2)
    k2     = ARM_L2 * math.sin(t2)
    t1     = math.atan2(y, x) - math.atan2(k2, k1)
    return math.degrees(t1), math.degrees(t2)

# ============================================================================
# SAMPLE G-CODE  (shows every supported command)
# ============================================================================

SAMPLE_GCODE = """\
G90          ; Absolute positioning
G20          ; Units: inches
G28          ; Home position
G0 X4 Y4     ; Rapid move to start
G1 X6 Y4 F2  ; Draw bottom edge
G1 X6 Y6     ; Draw right edge
G1 X4 Y6     ; Draw top edge
G1 X4 Y4     ; Draw left edge (close square)
G4 P1        ; Pause 1 second
G0 X6 Y5     ; Move to circle start
G2 X6 Y5 I-1 J0  ; Clockwise full circle (radius 1\", center at X5 Y5)
G3 X6 Y5 I-1 J0  ; Counter-clockwise full circle
G91          ; Switch to incremental mode
G1 X1 Y0 F2  ; Move 1\" right
G90          ; Back to absolute mode
G92 X0 Y0    ; Set current position as origin
G1 X2 Y2 F2  ; Move using new origin
M6           ; Tool change
M2           ; End program
"""

# ============================================================================
# GUI
# ============================================================================

class RobotControllerGUI:

    def __init__(self, root):
        self.root = root
        self.root.title(WINDOW_TITLE)
        self.root.geometry("1100x850")

        self.drawing_enabled = False
        self.last_x = None
        self.last_y = None
        self.drawing_path = []   # list of (robot_x, robot_y)

        self._build_ui()

        self.log_canvas(f"Canvas: {PAPER_WIDTH_INCHES}\" x {PAPER_HEIGHT_INCHES}\"")
        self.log_canvas(f"Max arm reach: {ARM_L1 + ARM_L2:.1f}\"")
        self.log_gcode("Ready — type G-code or draw, then click 'Save to moves.txt'")

    # ------------------------------------------------------------------
    # UI CONSTRUCTION
    # ------------------------------------------------------------------

    def _build_ui(self):
        # Top bar
        top = tk.Frame(self.root, bg="#2c3e50", height=60)
        top.pack(fill=tk.X)
        top.pack_propagate(False)

        tk.Label(top, text="Mode:", font=("Arial", 12, "bold"),
                 bg="#2c3e50", fg="white").pack(side=tk.LEFT, padx=15)

        self.mode_var = tk.StringVar(value="canvas")
        for label, val in [("Canvas Drawing", "canvas"),
                            ("G-code Commands", "gcode")]:
            tk.Radiobutton(top, text=label, variable=self.mode_var,
                           value=val, command=self._switch_mode,
                           font=("Arial", 11), bg="#2c3e50", fg="white",
                           selectcolor="#34495e",
                           activebackground="#34495e",
                           activeforeground="white").pack(side=tk.LEFT, padx=5)

        # Notebook tabs
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.tab_canvas = tk.Frame(self.nb)
        self.nb.add(self.tab_canvas, text="  Canvas Drawing  ")
        self._build_canvas_tab()

        self.tab_gcode = tk.Frame(self.nb)
        self.nb.add(self.tab_gcode, text="  G-code Commands  ")
        self._build_gcode_tab()

        # Status bar
        bar = tk.Frame(self.root, relief=tk.SUNKEN, borderwidth=2, bg="#ecf0f1")
        bar.pack(fill=tk.X, side=tk.BOTTOM, padx=5, pady=5)
        self.pos_label = tk.Label(
            bar, text="Position: X=0.00\" Y=0.00\" | θ1=0.0° θ2=0.0°",
            font=("Courier", 10), bg="#ecf0f1")
        self.pos_label.pack(side=tk.LEFT, padx=10)
        self.status_label = tk.Label(bar, text="Ready",
                                     font=("Arial", 10), bg="#ecf0f1")
        self.status_label.pack(side=tk.RIGHT, padx=10)

        self._switch_mode()

    # ---- Canvas tab ----

    def _build_canvas_tab(self):
        banner = tk.Frame(self.tab_canvas, bg="#3498db", height=50)
        banner.pack(fill=tk.X)
        banner.pack_propagate(False)
        tk.Label(banner,
                 text=f"Draw freehand ({PAPER_WIDTH_INCHES}\" x {PAPER_HEIGHT_INCHES}\" paper) "
                      f"then save as G-code to moves.txt",
                 font=("Arial", 11), bg="#3498db", fg="white").pack(pady=12)

        btns = tk.Frame(self.tab_canvas)
        btns.pack(pady=10)

        self.draw_btn = tk.Button(btns, text="Enable Drawing",
                                  command=self._toggle_draw,
                                  bg="#2ecc71", fg="white",
                                  font=("Arial", 10, "bold"),
                                  width=15, height=2)
        self.draw_btn.grid(row=0, column=0, padx=5)

        tk.Button(btns, text="Clear Canvas",
                  command=self._clear_canvas,
                  bg="#95a5a6", fg="white",
                  font=("Arial", 10), width=15, height=2
                  ).grid(row=0, column=1, padx=5)

        tk.Button(btns, text="Save to moves.txt",
                  command=self._export_canvas,
                  bg="#e67e22", fg="white",
                  font=("Arial", 10, "bold"),
                  width=15, height=2).grid(row=0, column=2, padx=5)

        frame = tk.Frame(self.tab_canvas, relief=tk.SUNKEN, borderwidth=3)
        frame.pack(padx=20, pady=5)
        self.canvas = tk.Canvas(frame, bg="white",
                                width=CANVAS_WIDTH, height=CANVAS_HEIGHT,
                                cursor="crosshair")
        self.canvas.pack()
        self._draw_workspace()

        self.canvas.bind("<Button-1>",        self._start_draw)
        self.canvas.bind("<B1-Motion>",       self._on_draw)
        self.canvas.bind("<ButtonRelease-1>", self._end_draw)

        log_f = tk.Frame(self.tab_canvas)
        log_f.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        tk.Label(log_f, text="Log:", font=("Arial", 10, "bold")).pack(anchor=tk.W)
        self.canvas_log = scrolledtext.ScrolledText(log_f, height=8,
                                                    font=("Courier", 9))
        self.canvas_log.pack(fill=tk.BOTH, expand=True)

    # ---- G-code tab ----

    def _build_gcode_tab(self):
        banner = tk.Frame(self.tab_gcode, bg="#9b59b6", height=50)
        banner.pack(fill=tk.X)
        banner.pack_propagate(False)
        tk.Label(banner,
                 text="Type G-code  →  Save to moves.txt  →  "
                      "Arduino reads and parses it directly",
                 font=("Arial", 11), bg="#9b59b6", fg="white").pack(pady=12)

        cols = tk.Frame(self.tab_gcode)
        cols.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # Left column: editor
        left = tk.Frame(cols)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        tk.Label(left, text="G-code Commands:",
                 font=("Arial", 10, "bold")).pack(anchor=tk.W)

        self.gcode_text = scrolledtext.ScrolledText(left, height=20,
                                                    font=("Courier", 10))
        self.gcode_text.pack(fill=tk.BOTH, expand=True, pady=5)
        self.gcode_text.insert(1.0, SAMPLE_GCODE)
        self.gcode_text.bind("<<Modified>>", self._sync_preview)

        btns = tk.Frame(left)
        btns.pack(pady=8)
        tk.Button(btns, text="Load File",
                  command=self._load_file,
                  width=14).grid(row=0, column=0, padx=4)
        tk.Button(btns, text="Save to moves.txt",
                  command=self._save_gcode,
                  bg="#e67e22", fg="white",
                  font=("Arial", 10, "bold"),
                  width=16).grid(row=0, column=1, padx=4)
        tk.Button(btns, text="Clear",
                  command=lambda: self.gcode_text.delete(1.0, tk.END),
                  width=8).grid(row=0, column=2, padx=4)

        # Right column: preview
        right = tk.Frame(cols)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))

        tk.Label(right,
                 text="moves.txt Preview  (what Arduino will receive):",
                 font=("Arial", 10, "bold")).pack(anchor=tk.W)

        self.preview = scrolledtext.ScrolledText(right, height=20,
                                                 font=("Courier", 9),
                                                 state=tk.DISABLED,
                                                 bg="#f8f8f8")
        self.preview.pack(fill=tk.BOTH, expand=True, pady=5)

        # Command reference
        ref = (
            "Commands:  G0 G1 G2 G3 G4  |  "
            "G20 G21 G28 G90 G91 G92  |  M2 M6\n"
            "Params:    X Y F (feed)  I J (arc centre offset)  P (dwell seconds)"
        )
        tk.Label(right, text=ref, font=("Courier", 8),
                 fg="#555", justify=tk.LEFT).pack(anchor=tk.W)

        # Log
        tk.Label(self.tab_gcode, text="Log:",
                 font=("Arial", 10, "bold")).pack(anchor=tk.W, padx=10)
        self.gcode_log = scrolledtext.ScrolledText(self.tab_gcode, height=6,
                                                   font=("Courier", 9))
        self.gcode_log.pack(fill=tk.X, padx=10, pady=5)

    # ------------------------------------------------------------------
    # MODE SWITCH
    # ------------------------------------------------------------------

    def _switch_mode(self):
        if self.mode_var.get() == "canvas":
            self.nb.select(self.tab_canvas)
            self.status_label.config(text="Canvas Drawing Mode")
        else:
            self.nb.select(self.tab_gcode)
            self.status_label.config(text="G-code Command Mode")

    # ------------------------------------------------------------------
    # CANVAS DRAWING
    # ------------------------------------------------------------------

    def _draw_workspace(self):
        cx, cy = CANVAS_WIDTH // 2, CANVAS_HEIGHT // 2
        r_px   = (ARM_L1 + ARM_L2) * PIXELS_PER_INCH
        self.canvas.create_oval(cx - r_px, cy - r_px,
                                cx + r_px, cy + r_px,
                                outline="#2ecc71", width=2, dash=(5, 5))
        self.canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3,
                                fill="#3498db", outline="")

    def _toggle_draw(self):
        self.drawing_enabled = not self.drawing_enabled
        if self.drawing_enabled:
            self.draw_btn.config(text="Disable Drawing", bg="#e74c3c")
            self.drawing_path = []
            self.log_canvas("Drawing enabled — drag mouse to draw")
        else:
            self.draw_btn.config(text="Enable Drawing", bg="#2ecc71")
            self.log_canvas("Drawing disabled")

    def _start_draw(self, event):
        if self.drawing_enabled:
            self.last_x, self.last_y = event.x, event.y

    def _on_draw(self, event):
        if not self.drawing_enabled or self.last_x is None:
            return
        self.canvas.create_line(self.last_x, self.last_y,
                                event.x, event.y,
                                width=BRUSH_SIZE, fill=DRAW_COLOR,
                                capstyle=tk.ROUND, smooth=True)
        rx, ry = canvas_to_robot(event.x, event.y)
        self.drawing_path.append((rx, ry))
        try:
            t1, t2 = inverse_kinematics(rx, ry)
            self.pos_label.config(
                text=f"Position: X={rx:.2f}\" Y={ry:.2f}\" | "
                     f"θ1={t1:.1f}° θ2={t2:.1f}°")
        except ValueError:
            pass
        self.last_x, self.last_y = event.x, event.y

    def _end_draw(self, event):
        self.last_x = self.last_y = None
        if self.drawing_path:
            self.log_canvas(f"Captured {len(self.drawing_path)} points")

    def _clear_canvas(self):
        self.canvas.delete("all")
        self.drawing_path = []
        self._draw_workspace()
        self.log_canvas("Canvas cleared")

    def _export_canvas(self):
        """Convert mouse path → G-code → save to moves.txt."""
        if not self.drawing_path:
            messagebox.showwarning("No Drawing", "Draw something on the canvas first!")
            return
        x0, y0 = self.drawing_path[0]
        lines = [
            "G90",
            "G20",
            "G28",
            f"G0 X{x0:.3f} Y{y0:.3f}",
        ]
        for x, y in self.drawing_path[1:]:
            lines.append(f"G1 X{x:.3f} Y{y:.3f} F{DEFAULT_FEED_RATE}")
        lines += ["G28", "M2"]
        self._write_moves_txt("\n".join(lines))

    # ------------------------------------------------------------------
    # G-CODE TAB
    # ------------------------------------------------------------------

    def _sync_preview(self, _event=None):
        """Keep right-hand preview in sync with the editor."""
        self.gcode_text.edit_modified(False)
        content = self.gcode_text.get(1.0, tk.END)
        self.preview.config(state=tk.NORMAL)
        self.preview.delete(1.0, tk.END)
        self.preview.insert(1.0, content)
        self.preview.config(state=tk.DISABLED)

    def _load_file(self):
        path = filedialog.askopenfilename(
            title="Select G-code / text file",
            filetypes=[("G-code / Text", "*.gcode *.nc *.txt"),
                       ("All files", "*.*")])
        if path:
            try:
                with open(path, "r") as f:
                    self.gcode_text.delete(1.0, tk.END)
                    self.gcode_text.insert(1.0, f.read())
                self.log_gcode(f"Loaded: {path}")
            except Exception as e:
                messagebox.showerror("Load Error", str(e))

    def _save_gcode(self):
        """Save raw G-code text directly to moves.txt — no conversion."""
        text = self.gcode_text.get(1.0, tk.END).strip()
        if not text:
            messagebox.showwarning("Empty", "Nothing to save.")
            return
        self._write_moves_txt(text)

    # ------------------------------------------------------------------
    # SHARED SAVE HELPER
    # ------------------------------------------------------------------

    def _write_moves_txt(self, gcode_text):
        """
        Write raw G-code to moves.txt on disk.
        The Arduino will read this file and parse the commands itself.
        """
        path = filedialog.asksaveasfilename(
            title="Save moves.txt for SD card",
            initialfile="moves.txt",
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if not path:
            return
        try:
            with open(path, "w") as f:
                f.write(gcode_text)
            n = len([l for l in gcode_text.splitlines() if l.strip()])
            self.log_gcode(f"Saved {n} lines → {path}")
            messagebox.showinfo(
                "Saved",
                f"G-code saved to:\n{path}\n\n"
                "Copy this file to your SD card as  'moves.txt'\n"
                "The Arduino will parse the G-code directly.")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    # ------------------------------------------------------------------
    # UTILITIES
    # ------------------------------------------------------------------

    def log_canvas(self, msg):
        self.canvas_log.insert(tk.END, msg + "\n")
        self.canvas_log.see(tk.END)

    def log_gcode(self, msg):
        self.gcode_log.insert(tk.END, msg + "\n")
        self.gcode_log.see(tk.END)

# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControllerGUI(root)
    root.mainloop()