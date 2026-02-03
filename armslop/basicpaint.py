import tkinter as tk
import math

BRUSH_SIZE = 5
COLOR = "black"

root = tk.Tk()
root.title("Tiny Paint")

#canvas scalled to 8.5 by 11 by 50%
canvas = tk.Canvas(root, bg="white", width=566, height=733)
canvas.pack()

cartisian_x, last_y = None, None
cartisian_y = None

def compute_angles() :
    if cartisian_x is None or cartisian_y is None:
        return

    r = math.sqrt(cartisian_x**2 + cartisian_y**2)
    theta = math.atan2(cartisian_y, cartisian_x)

    angle1 = math.acos(r / 800) + theta
    angle2 = (math.pi/2 - (angle1 - theta)) * 2

    angle1= math.degrees(angle1)
    angle1=round(angle1)

    angle2 = math.degrees(angle2)
    angle2 = round(angle2)
    print(angle1,',',angle2,sep='')

def start_draw(event):
    global cartisian_x, last_y
    cartisian_x, last_y = event.x, event.y

def draw(event):
    global cartisian_x, last_y, cartisian_y
    canvas.create_line(
        cartisian_x, last_y, event.x, event.y,
        width=BRUSH_SIZE, fill=COLOR, capstyle=tk.ROUND
    )
    cartisian_x, last_y = event.x, event.y
    cartisian_y = 733-last_y
    cartisian_x = int(cartisian_x)
    cartisian_y = int(cartisian_y)
    compute_angles()

    
  
    

canvas.bind("<Button-1>", start_draw)
canvas.bind("<B1-Motion>", draw)

root.mainloop()
