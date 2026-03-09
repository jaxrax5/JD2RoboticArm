import tkinter as tk
from tkinter import scrolledtext, filedialog, messagebox
import math

BRUSH_SIZE  = 5
COLOR       = "black"
CANVAS_W    = 566
CANVAS_H    = 733
FEED_RATE   = 4.0   # inches/sec — matches engineering requirement

print("G90")

# ─── State ────────────────────────────────────────────────────────────────────
cartisian_x, last_y = None, None
cartisian_y  = None
gcode_lines  = []
line_start   = None
preview_id   = None
mode         = "freehand"   # "freehand" | "line"

# ─── Root ─────────────────────────────────────────────────────────────────────
root = tk.Tk()
root.title("Tiny Paint — G-code")
root.configure(bg="#2c3e50")
root.resizable(False, False)

# ─── Banner ───────────────────────────────────────────────────────────────────
banner = tk.Frame(root, bg="#2c3e50", height=50)
banner.pack(fill=tk.X)
banner.pack_propagate(False)
tk.Label(banner, text="🖊  Tiny Paint  →  G-code",
         font=("Arial", 13, "bold"), bg="#2c3e50", fg="white"
         ).pack(side=tk.LEFT, padx=16, pady=8)

# ─── Toolbar ──────────────────────────────────────────────────────────────────
toolbar = tk.Frame(root, bg="#34495e", pady=6)
toolbar.pack(fill=tk.X)

def set_mode(m):
    global mode
    mode = m
    fb.config(relief=tk.SUNKEN if m == "freehand" else tk.FLAT)
    lb.config(relief=tk.SUNKEN if m == "line"     else tk.FLAT)

fb = tk.Button(toolbar, text="✏ Freehand", command=lambda: set_mode("freehand"),
               bg="#2980b9", fg="white", font=("Arial", 10, "bold"),
               width=12, relief=tk.SUNKEN, cursor="hand2")
fb.pack(side=tk.LEFT, padx=(10, 3))

lb = tk.Button(toolbar, text="📏 Line", command=lambda: set_mode("line"),
               bg="#2980b9", fg="white", font=("Arial", 10),
               width=10, relief=tk.FLAT, cursor="hand2")
lb.pack(side=tk.LEFT, padx=3)

tk.Frame(toolbar, bg="#7f8c8d", width=2).pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=4)

def clear_canvas():
    global gcode_lines, preview_id, line_start
    canvas.delete("all")
    gcode_lines = []
    preview_id = line_start = None
    _log("--- canvas cleared ---")

def save_moves():
    # Only G1 lines are in gcode_lines — no G90/G28/M2 that would move arm to 0,0
    if not gcode_lines:
        messagebox.showwarning("Nothing to save", "Draw something first!")
        return
    # Apply paper offset so arm origin maps to paper corner
    try:
        ox = int(offset_x.get())
        oy = int(offset_y.get())
    except ValueError:
        messagebox.showerror("Bad offset", "Offsets must be integers.")
        return
    path = filedialog.asksaveasfilename(
        title="Save moves.txt", initialfile="moves.txt",
        defaultextension=".txt",
        filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
    if not path:
        return
    lines = []
    for i, g in enumerate(gcode_lines):
        parts = g.split()
        x = int(parts[1][1:]) + ox
        y = int(parts[2][1:]) + oy
        # First point is always G0 (rapid pen-up travel to start)
        # so the arm doesn't drag the pen across the page to reach position
        cmd = "G0" if i == 0 else "G1"
        f_str = f" F{FEED_RATE}" if cmd == "G1" else ""
        lines.append(f"{cmd} X{x} Y{y}{f_str}")
    lines.append("M2")   # signal Arduino we are done
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    _log(f"Saved {len(lines)} lines (G0 start + G1 draw + M2) → {path}")
    messagebox.showinfo("Saved", f"{len(lines)} lines saved to:\n{path}")

tk.Button(toolbar, text="🗑 Clear", command=clear_canvas,
          bg="#95a5a6", fg="white", font=("Arial", 10),
          width=10, relief=tk.FLAT, cursor="hand2").pack(side=tk.LEFT, padx=3)

tk.Button(toolbar, text="💾 Save moves.txt", command=save_moves,
          bg="#e67e22", fg="white", font=("Arial", 10, "bold"),
          width=16, relief=tk.FLAT, cursor="hand2").pack(side=tk.LEFT, padx=3)

# Offset controls — shift all coordinates so paper corner = arm origin
tk.Frame(toolbar, bg="#7f8c8d", width=2).pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=4)
tk.Label(toolbar, text="Paper offset  X:", bg="#34495e", fg="white",
         font=("Arial", 9)).pack(side=tk.LEFT)
offset_x = tk.Entry(toolbar, width=5, font=("Arial", 10))
offset_x.insert(0, "0")
offset_x.pack(side=tk.LEFT, padx=2)
tk.Label(toolbar, text="Y:", bg="#34495e", fg="white",
         font=("Arial", 9)).pack(side=tk.LEFT)
offset_y = tk.Entry(toolbar, width=5, font=("Arial", 10))
offset_y.insert(0, "0")
offset_y.pack(side=tk.LEFT, padx=2)

# ─── Canvas ───────────────────────────────────────────────────────────────────
canvas_border = tk.Frame(root, bg="#7f8c8d", padx=2, pady=2)
canvas_border.pack(padx=14, pady=8)
canvas = tk.Canvas(canvas_border, bg="white",
                   width=CANVAS_W, height=CANVAS_H, cursor="crosshair")
canvas.pack()

# ─── Status bar ───────────────────────────────────────────────────────────────
statusbar = tk.Frame(root, bg="#ecf0f1", relief=tk.SUNKEN, borderwidth=1)
statusbar.pack(fill=tk.X, padx=14)
pos_label = tk.Label(statusbar, text="X=0  Y=0",
                     font=("Courier", 9), bg="#ecf0f1", fg="#2c3e50")
pos_label.pack(side=tk.LEFT, padx=8, pady=2)
len_label = tk.Label(statusbar, text="", font=("Courier", 9, "bold"),
                     bg="#ecf0f1", fg="#c0392b")
len_label.pack(side=tk.RIGHT, padx=8)

# ─── Log ──────────────────────────────────────────────────────────────────────
log_frame = tk.Frame(root, bg="#2c3e50")
log_frame.pack(fill=tk.X, padx=14, pady=(4, 12))
tk.Label(log_frame, text="G-code output:", font=("Arial", 9, "bold"),
         bg="#2c3e50", fg="#bdc3c7").pack(anchor=tk.W)
log_box = scrolledtext.ScrolledText(log_frame, height=8, font=("Courier", 9),
                                    bg="#1a252f", fg="#2ecc71", state=tk.DISABLED)
log_box.pack(fill=tk.X)

def _log(msg):
    log_box.config(state=tk.NORMAL)
    log_box.insert(tk.END, msg + "\n")
    log_box.see(tk.END)
    log_box.config(state=tk.DISABLED)

def _record(gx, gy):
    """Append one G1 command — the ONLY thing written to moves.txt."""
    gcode = f"G1 X{gx} Y{gy}"
    print(gcode)
    gcode_lines.append(gcode)
    _log(gcode)
    pos_label.config(text=f"X={gx}  Y={gy}")

# ─── Original freehand logic (untouched) ──────────────────────────────────────
def start_draw(event):
    global cartisian_x, last_y, line_start, preview_id
    if mode == "freehand":
        cartisian_x, last_y = event.x, event.y
    else:
        line_start = (event.x, event.y)

def draw(event):
    global cartisian_x, last_y, cartisian_y, preview_id
    if mode == "freehand":
        canvas.create_line(cartisian_x, last_y, event.x, event.y,
                           width=BRUSH_SIZE, fill=COLOR, capstyle=tk.ROUND)
        cartisian_x, last_y = event.x, event.y
        cartisian_y = CANVAS_H - last_y
        cartisian_x = int(cartisian_x)
        cartisian_y = int(cartisian_y)
        _record(cartisian_x, cartisian_y)
    else:
        # Live snapped preview for line tool
        if line_start is None:
            return
        sx, sy = line_start
        ex, ey = event.x, event.y
        # Snap to horizontal/vertical within 15°
        dx, dy = ex - sx, ey - sy
        if dx != 0 or dy != 0:
            ang = abs(math.degrees(math.atan2(dy, dx)))
            if ang <= 15 or ang >= 165:
                ey = sy
            elif abs(ang - 90) <= 15:
                ex = sx
        length_px = math.hypot(ex - sx, ey - sy)
        PX_PER_IN = CANVAS_W / 8.5
        length_in = length_px / PX_PER_IN
        len_label.config(text=f"{length_in:.2f}\"  {'✓ 10in' if abs(length_in - 10) <= 0.25 else ''}")
        if preview_id:
            canvas.delete(preview_id)
        preview_id = canvas.create_line(sx, sy, ex, ey, width=BRUSH_SIZE,
                                        fill="#3498db", dash=(6, 4), capstyle=tk.ROUND)

def end_draw(event):
    global line_start, preview_id
    if mode == "line" and line_start is not None:
        sx, sy = line_start
        ex, ey = event.x, event.y
        # Apply same snap
        dx, dy = ex - sx, ey - sy
        if dx != 0 or dy != 0:
            ang = abs(math.degrees(math.atan2(dy, dx)))
            if ang <= 15 or ang >= 165:
                ey = sy
            elif abs(ang - 90) <= 15:
                ex = sx
        if preview_id:
            canvas.delete(preview_id)
            preview_id = None
        canvas.create_line(sx, sy, ex, ey, width=BRUSH_SIZE,
                           fill=COLOR, capstyle=tk.ROUND)
        # Bresenham-style: one G1 per integer pixel — guarantees every step
        ix0, iy0 = int(round(sx)), int(round(sy))
        ix1, iy1 = int(round(ex)), int(round(ey))
        ddx, ddy  = ix1 - ix0, iy1 - iy0
        steps     = max(abs(ddx), abs(ddy), 1)
        for i in range(steps + 1):
            gx = ix0 + int(round(ddx * i / steps))
            gy = CANVAS_H - (iy0 + int(round(ddy * i / steps)))
            _record(gx, gy)
        line_start = None
        len_label.config(text="")

canvas.bind("<Button-1>",        start_draw)
canvas.bind("<B1-Motion>",       draw)
canvas.bind("<ButtonRelease-1>", end_draw)

root.mainloop()