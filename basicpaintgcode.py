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
print("G90")



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
    print(f"G1 X{cartisian_x} Y{cartisian_y}")
  

    
  
    

canvas.bind("<Button-1>", start_draw)
canvas.bind("<B1-Motion>", draw)

root.mainloop()