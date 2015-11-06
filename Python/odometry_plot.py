import matplotlib.pyplot as plt
import math
import numpy as np

class OdometryPlot():


    def __init__(self):
        # Setup matplotlib figure

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.lines, = self.ax.plot([],[])
        self.ax.set_autoscale_on(True)
        self.xs = []
        self.ys = []

    @staticmethod
    def calc_odometry(counters, old_counters, x, y, theta):
        BODY_LENGTH = 52.7
        counters_step = counters - old_counters

        arcs = counters_step * 0.08

        distance = 0.5 * (arcs[0] + arcs[1])
        theta = theta + (arcs[1] - arcs[0])/BODY_LENGTH
        theta = theta % (2 * math.pi)

        x = x + distance * math.cos(theta)
        y = y + distance * math.sin(theta)

        # angle home
        phi = np.pi + math.atan2(y,x)

        return x, y, theta, phi

    def update(self, x, y):
        self.xs.append(x)
        self.ys.append(y)

        self.lines.set_xdata(self.xs)
        self.lines.set_ydata(self.ys)

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

