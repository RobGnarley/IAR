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
        #set_counts(self.connection, 0, 0)

        # INTERNAL STATE VARS
        #self.turning = None
        self.error = 0
        self.v_left = self.WALL_SPEED
        self.v_right = self.WALL_SPEED

        # ODOMETRY VARS
        #x = 0.0
        #y = 0.0
        #theta = 0.0
        #old_counters = read_counts(s)
       # total_distance = 0.0
        

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
        fig, ax = plt.subplots()
        lines, = ax.plot([],[])
        ax.set_autoscale_on(True)
        xs = []
        ys = []

    def update(self, x, y):
        xs.append(x)
        ys.append(y)

        lines.set_xdata(xs)
        lines.set_ydata(ys)

        ax.relim()
        ax.autoscale_view()

        fig.canvas.draw()
        fig.canvas.flush_events()

if __name__ == '__main__':
    robot = kheperam()    
    try:
        while True:
            robot.update()
    except KeyboardInterrupt:
        stop(robot.connection)
        sys.exit()
