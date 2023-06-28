"""
NOT A ROS2 NODE
Plotting for 360° range sensor data

NOTE: The turtlebot3 can come with 2 different LiDAR sensors
(LDS01, older robots and simulation vs LDS02, newer robots).
Those two packages order LiDAR data differently, which may
cause inconsistencies in orientation.
"""
import math

import tkinter as tk
import numpy as np

# pylint: disable=C0103
class SensorPlotter:
    """ Plotter class """
    def __init__(self) -> None:
        self.width = 600
        self.height = self.width
        self.radius = 260
        self.window = tk.Tk()
        self.canvas = tk.Canvas(self.window, width=self.width, height=self.height, bg="white")
        self.canvas.grid(row=1,column=1)
        self.points = []

        # Initialize canvas items
        h,w = 5,4 # height and width of central triangle
        x1, y1 = self.width/2, self.height/2 - h
        x2, y2 = self.width/2 - w, self.height/2 + h
        x3, y3 = self.width/2 + w, self.height/2 + h
        self.canvas.create_polygon(x1, y1, x2, y2, x3, y3, fill="#44ff44", outline='black')

        #@TODO: init count here instead.
        self.window.update()


    def draw_rays(self, distances: list, maxrange=1):
        """ Draws rays from a list of distances """
        if len(self.points) == 0:
            self.__init_points(raycount=len(distances))

        distances = np.array(distances) / maxrange
        distances[distances == np.inf] = 1. # Treat inf values as maximum range
        n_acq = len(distances)
        angle = 0
        angle_step = 2*math.pi/n_acq

        x_center = self.width/2
        y_center = self.height/2
        for f, point in zip(distances, self.points):
            pt_x = x_center - f*self.radius*math.sin(angle)
            pt_y = y_center - f*self.radius*math.cos(angle)
            if f == 1.:
                color = 'red'
            else:
                color = '#55ff55'
            self.__move_point(point, pt_x, pt_y, color)
            angle += angle_step
        self.window.update()


    def draw_rays_180(self, distances: list, maxrange=1):
        """
        Draws only forward rays from a list of distances
        NOTE: may be broken depending on sensor.
        """
        if len(self.points) == 0:
            self.__init_points(raycount=len(distances))

        distances = np.array(distances) / maxrange
        distances = np.concatenate(distances[270:], distances[:90]) # NOTE: SENSOR-DEPENDANT!!!
        distances[distances == np.inf] = 1. # Treat inf values as maximum range
        n = len(distances)
        angle = 0
        angle_step = math.pi/n

        x_center = self.width/2
        y_center = self.height/2
        for f, point in zip(distances, self.points):
            pt_x = x_center + f*self.radius*math.cos(angle)
            pt_y = y_center - f*self.radius*math.sin(angle)
            if f == 1.:
                color = 'red'
            else:
                color = '#55ff55'
            self.__move_point(point, pt_x, pt_y, color)
            angle += angle_step
        self.window.update()


    def __init_points(self, raycount: int):
        for _ in range(0, raycount): # measurement points
            self.points.append(self.__plot_point(self.width/2, self.height/2,  '#55ff55', radius=2))


    def __plot_point(self, x: int, y: int, color: str, radius: int = 1):
        return self.canvas.create_oval(x - radius, y - radius, x + radius,y + radius,
                                       fill=color, outline='blue')


    def __move_point(self, point, x: int, y: int,
    color: str = ''):
        if color == 'red':
            outline = ''
        else:
            outline = 'blue'
        self.canvas.moveto(point, x, y)
        self.canvas.itemconfig(point, fill=color, outline=outline)
