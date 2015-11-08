import matplotlib.pyplot as plt
import math
import numpy as np
from collections import namedtuple
from mapping import grid as occupancy_grid

Point = namedtuple('Point', ['x','y'])

class OdometryPlot():


    def __init__(self, use_map=False):
        # Setup matplotlib figure
        self.use_map = use_map
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.lines, = self.ax.plot([],[])
        self.xs = []
        self.ys = []
        if use_map:
            self.ax.imshow(occupancy_grid, cmap='Greys', interpolation='None')
        else:
            self.ax.set_autoscale_on(True)

    @staticmethod
    def calc_odometry(counters, old_counters, x, y, theta, target=Point(0,0)):
        BODY_LENGTH = 52.7
        counters_step = counters - old_counters

        arcs = counters_step * 0.08

        distance = 0.5 * (arcs[0] + arcs[1])
        theta = theta + (arcs[1] - arcs[0])/BODY_LENGTH
        theta = theta % (2 * math.pi)

        x = x + distance * math.cos(theta)
        y = y + distance * math.sin(theta)

        # angle home
        phi = np.pi + math.atan2(y-target.y,x-target.x)

        return x, y, theta, phi

    def update(self, x, y):
        if self.use_map:
            y = y/(770/30.0)
            x = x/(1400/52.0)
            print x, y
            x = x + 25
            y = y + 25
        else:
            self.ax.relim()
            self.ax.autoscale_view()

        self.xs.append(x)
        self.ys.append(y)

        self.lines.set_xdata(self.xs)
        self.lines.set_ydata(self.ys)


        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

