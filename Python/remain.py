from khepera_functions import *
import math
import matplotlib.pyplot as plt
import numpy as np
import logging
import sys

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.DEBUG)
logger.addHandler(ch)

class kheperam():

    def __init__(self):
        STATES = ['BEGIN', 'EXPLORE', 'AVOID', 'FOLLOW_WALL', 'STOP']
        self.EXPLORE_SPEED = 6
        self.TURN_SPEED = 3
        self.WALL_SPEED = 3
        # If we are closer than 500 then there is very little distance for the robot to try to get closer 
        # while there is a lot of distance where it is "too close"
        # 700 might be a good value
        self.IDEAL_IR = 400
        #K1 = 0.001
        #K2 = 0.1
        #K1 = 0.0000005
        #K2 = 0.0001
        self.K1 = 0.0001
        self.K2 = 0.001

        self.state = 'EXPLORING'

        #self.WHEEL_RADIUS = 0.08
        #self.BODY_LENGTH = 52.7
        #self.EXPLORE_TIME = 90
        self.connection = open_connection()
        set_counts(self.connection, 0, 0)

        # INTERNAL STATE VARS
        #self.turning = None
        self.error = 0
        self.v_left = self.WALL_SPEED
        self.v_right = self.WALL_SPEED
        
        self.old_counters = [0,0]
        self.x = 0
        self.y = 0
        self.theta = 0
        self.graph = OdometryPlot()
        self.graph.update(self.x, self.y) 

    def explore(self):
        go(self.connection, self.EXPLORE_SPEED)

    def follow_wall(self, ir_sensors):
        if self.state != 'FOLLOW WALL':
            self.v_left = self.WALL_SPEED
            self.v_right = self.WALL_SPEED
            self.state = 'FOLLOW WALL'
        if ir_sensors[0] > ir_sensors[5]:
                wall = 'left'
                sensor = ir_sensors[0]
        else:
                wall = 'right'
                sensor = ir_sensors[5]

        self.old_error = self.error
        self.error = self.IDEAL_IR - sensor
        delta_error = self.error - self.old_error

        # Negative error - too close, drive away
        # Postive error  - too far away, drive closer
        delta_v = (self.K1 * self.error) + (self.K2 * delta_error)
        
        if wall == 'left':
            self.v_right = self.v_right + delta_v
        else:
            self.v_left = self.v_left + delta_v


        turn(self.connection,int(self.v_left),self.v_right)


        if sensor < 10:
        # Lost wall, go back to exploring
            self.v_left = self.WALL_SPEED
            self.state = 'EXPLORING'
 

    def avoid(self, ir_sensors):
        """call this when an obstacle is detected. It will set the current state to avoiding.
        once the correct sensor readings are found it will leave the state into the generic explore state """
        if self.state != 'AVOIDING':
            self.state == 'AVOIDNING'
            if ir_sensors[2] > ir_sensors[3]: # We could go back to using a more clever choice here
                # Turn right
                turn(self.connection,self.TURN_SPEED,-self.TURN_SPEED)
            else:
                # Turn left
                turn(self.connection,-self.TURN_SPEED,self.TURN_SPEED)
        if (ir_sensors[1] < 50 and ir_sensors[4] < 50) and ir_sensors[2] < 70 and ir_sensors[3] < 70:
            self.state = 'EXPLORING'



    def update(self):
        try:
            ir_sensors, counters = self.get_readings()
        except ValueError as e:
            logger.debug(e)
            return 
        
        self.x, self.y, self.theta = OdometryPlot.calc_odometry(counters, self.old_counters, self.x, self.y, self.theta)       

        self.graph.update(self.x, self.y)    

        self.old_counters = counters

        if ir_sensors[2] > 100 or ir_sensors[3] > 100 or self.state == 'AVOIDING':
            self.avoid(ir_sensors)

        elif ir_sensors[0] > 50 or ir_sensors[5] > 50 or self.state == 'FOLLOW WALL':
            self.follow_wall(ir_sensors)

        else:
            self.explore() 

    def get_readings(self):

        ir_sensors = read_IR(self.connection) 
        counters = np.array([float(i) for i in read_counts(self.connection)])
        return ir_sensors, counters

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
        
        return x, y, theta
 

    def update(self, x, y):
        self.xs.append(x)
        self.ys.append(y)

        self.lines.set_xdata(self.xs)
        self.lines.set_ydata(self.ys)
        
        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

if __name__ == '__main__':
    robot = kheperam()    
    try:
        while True:
            robot.update()
    except KeyboardInterrupt:
        stop(robot.connection)
        sys.exit()

    except Exception as e:
        logger.warning(e)
        stop(robot.connection)
        sys.exit()
