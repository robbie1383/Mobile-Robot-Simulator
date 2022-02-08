import time
from tkinter import *

canvas_width = 1000
canvas_height = 800
blank = 50
room_outer = [[50, 50], [50, 750], [950, 750], [950, 50], [50, 50]]  # outer edge points,could be used for sensor model
room_inner = [[450, 350], [450, 450], [550, 450], [550, 350], [450, 350]]  # room inner edge points
environment = Tk()
environment.title("Mobile-Root-Simulator")
canvas = Canvas(environment, width=canvas_width, height=canvas_height)
canvas.pack()
canvas.create_line(room_outer, width=2)  # draw the outer wall
canvas.create_line(room_inner, width=2)  # draw the inner wall

x = 100
y = 100
r = 50
robot = canvas.create_oval([x - r, x - r], [x + r, x + r], fill='pink')
robot_direction = canvas.create_oval([x, y], [x, y + r], width=2)
it = 0  # just for show moving,should be removed finally
update_time = 1

while True:
    it += 1
    x += 3
    y += 5
    canvas.move(robot, 3, 5)
    canvas.delete(robot_direction)
    # -----------------------
    # Todo : call the robot.move function
    # Todo: the keyboard events
    b = it % 4
    if b == 0:
        robot_direction = canvas.create_oval(x, y, x, y - r, width=2)
    elif b == 1:
        robot_direction = canvas.create_oval(x, y, x, y + r, width=2)
    elif b == 2:
        robot_direction = canvas.create_oval(x, y, x - r, y, width=2)
    elif b == 3:
        robot_direction = canvas.create_oval(x, y, x + r, y, width=2)
    # -----------------------
    canvas.update()
    time.sleep(update_time)
