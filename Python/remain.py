from khepera_functions import *
import math
import numpy as np
import logging
import sys
import time
from odometry_plot import OdometryPlot
from collections import namedtuple

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.DEBUG)
logger.addHandler(ch)

Point = namedtuple('Point', ['x', 'y'])

class Kheperam(object):

    def __init__(self):
        STATES = ['BEGIN', 'EXPLORE', 'AVOID', 'FOLLOW_WALL', 'STOP', 'GETTING FOOD','GOING HOME']
        self.EXPLORE_SPEED = 6
        self.TURN_SPEED = 3
        self.WALL_SPEED = 3
        # If we are closer than 500 then there is very little distance for the robot to try to get closer 
        # while there is a lot of distance where it is "too close"
        # 700 might be a good value
        self.IDEAL_IR = 700
        self.K1 = 0.0001
        self.K2 = 0.019

        self.state = 'EXPLORING'

        self.connection = open_connection()
        set_counts(self.connection, 0, 0)

        # INTERNAL STATE VARS
        self.error = 0
        self.v_left = self.WALL_SPEED
        self.v_right = self.WALL_SPEED
        
        self.old_counters = [0,0]
        self.ir_sensors = []
        self.distances = []
        # wall is either 'left', 'right' or None
        self.wall = None
        self.x = 0
        self.y = 0
        # current angle
        self.theta = 0
        # angle to home
        self.phi = np.pi
        self.graph = OdometryPlot(use_map = True)
        self.graph.update(self.x, self.y)
        # Time elapsed since journey started
        self.clock = 0
        # Belief that robot is pointing home
        self.pointing_home = False

        self.ambient = read_ambient(self.connection)
        self.graph.fig.canvas.mpl_connect('key_press_event', self.key_pressed)

        self.has_food = False
        self.food_at = None
        self.home = Point(x=0, y=0)
        self.target = self.home
        self.turning_home = False
        self.avoiding = False

        self.avoided_counter = 0

    def key_pressed(self, event):
        self.state = 'GOING HOME'
        self.has_food = True
        self.food_at = Point(x=self.x, y=self.y)
        self.target = self.home
        self.blink()

    def run(self, test=0):
        """
        Main control loop
        """
        START_TIME = time.time()
        
        if test == 0:
            while True:
                CURRENT_TIME = time.time()
                self.clock = CURRENT_TIME - START_TIME
                try:
                    self.update()
                except KeyboardInterrupt:
                    stop(self.connection)
                    break
                except Exception as e:
                    stop(self.connection)
                    raise e
        else:
            # Use for testing
            #while True:
            #    try:
            #        self.update_vars()
            #        self.point_home()
            #    except KeyboardInterrupt:
            #        stop(self.connection)
            #        #sys.exit()
            #        break
            ir = self.update_vars()
            
    def explore(self):
        go(self.connection, self.EXPLORE_SPEED)

    def follow_wall(self, ir_sensors):
        if self.state != 'FOLLOW WALL':
            self.v_left = self.WALL_SPEED
            self.v_right = self.WALL_SPEED
            if self.state == 'EXPLORING':
                self.state = 'FOLLOW WALL' 
        if ir_sensors[0] > ir_sensors[5]:
                if not self.wall:
                    self.wall = 'left'
                sensor = ir_sensors[0]
        else:
                if not self.wall:
                    self.wall = 'right'
                sensor = ir_sensors[5]

        self.old_error = self.error
        self.error = self.IDEAL_IR - sensor
        delta_error = self.error - self.old_error

        # Negative error - too close, drive away
        # Postive error  - too far away, drive closer
        delta_v = (self.K1 * self.error) + (self.K2 * delta_error)
        
        if self.wall == 'left':
            self.v_right = self.v_right + delta_v
        else:
            self.v_left = self.v_left + delta_v


        turn(self.connection,int(self.v_left),self.v_right)


        if sensor < 10:
        # Lost wall, go back to exploring
            self.v_left = self.WALL_SPEED
            self.wall = None
            self.state = 'EXPLORING'
 

    def avoid(self, ir_sensors, distances):
        """call this when an obstacle is detected. It will set the current state to avoiding.
        once the correct sensor readings are found it will leave the state into the generic explore state """
        if not self.avoiding:
            self.avoiding = True
            if sum(distances[0:2]) < sum(distances[3:5]): # We could go back to using a more clever choice here
                # Turn right
                turn(self.connection,self.TURN_SPEED,-self.TURN_SPEED)
            else:
                # Turn left
                turn(self.connection,-self.TURN_SPEED,self.TURN_SPEED)
        if self.logic_or([x > 50  for x in distances[1:4]]):
            self.avoiding = False
            self.avoided_counter = 5

    def update_vars(self):
        try:
            ir_sensors, counters, self.ambient = self.get_readings()
        except ValueError as e:
            logger.debug(e)
            return self.update_vars()
        
        self.x, self.y, self.theta, self.phi = OdometryPlot.calc_odometry(
                counters, self.old_counters, self.x, self.y, self.theta,self.target)   

        self.old_counters = counters

        self.ir_sensors = ir_sensors

        self.distances = self.get_distances(ir_sensors)

    def update(self):
        
        self.update_vars()
        ir_sensors = self.ir_sensors
        distances = self.distances

        self.graph.update(self.x, self.y)

        #if self.avoided_counter > 0:
        #    self.avoided_counter -= 1
        if (self.detect_ahead() or self.avoiding) and not self.turning_home:
            self.avoid(ir_sensors, distances)
            print 'AVOIDING'
        elif self.has_food or self.state == 'GETTING FOOD':
            self.return_home(ir_sensors,distances)
        #elif ir_sensors[0] > 50 or ir_sensors[5] > 50 or self.state == 'FOLLOW WALL':
        elif self.logic_or([x < 100 for x in distances]):
            self.follow_wall(ir_sensors)
        else:
            self.state = 'EXPLORING'
            self.explore()

        logger.info(self.state)
        logger.debug(self.wall)

    def logic_or(self,xs):

        for x in xs:
            if x:
                return True

        return False

    def detect_ahead(self):

        return self.logic_or([x < 50 for x in self.distances[1:4]])

    def point_home(self):
        """
        Turn on the spot until pointing at home.
        """

        self.turning_home = True
        
        error = (self.theta - self.phi) % (2*math.pi)

        if error < (math.pi/16):
            stop(self.connection)
            self.turning_home = False
            raise DoneTurning('We have turned to within the error')

        if error > (math.pi / 2) and error < (3*math.pi/2):
            turn_v = 3
        else:
            turn_v = 2 

        if error > math.pi:
            # turn left
            turn(self.connection,-turn_v,turn_v)
        else:
            # turn right
            turn(self.connection,turn_v,-turn_v)


    def return_home(self, ir_sensors, distances):

        dir_target = 'left' if self.phi > math.pi else 'right'

        #if (self.phi > 3 * math.pi / 4.0 and self.phi < (5*math.pi) / 4.0) or self.phi < math.pi / 4.0 or self.phi > 7 * math.pi / 4.0 :
        #    dir_target = None

        print 'dir_target: ' + str(dir_target)

        if (dir_target == 'left' and (distances[0] < 100 or distances[1] < 100) ) or (dir_target == 'right' and (distances[4] < 100 or distances[5] < 100)):
            self.follow_wall(ir_sensors)
            print 'FOLLOWED WALL WITH FOOD'
            return

        self.wall = None

        distance_error = abs(self.target.x - self.x) + abs(self.target.y - self.y)
        if distance_error < 5 and self.has_food:
            self.blink()
            self.has_food = False
            self.target = self.food_at
            self.state = 'GETTING FOOD'
            return
        elif distance_error < 5:
            self.state = 'EXPLORING'
            return
        try:
            self.point_home()
        except DoneTurning:
            go(self.connection, self.EXPLORE_SPEED)  

    def get_readings(self):

        ir_sensors = read_IR(self.connection) 
        counters = np.array([float(i) for i in read_counts(self.connection)])
        amb = read_ambient(self.connection)
        return ir_sensors, counters, amb

    def stop(self):
        stop(self.connection)
        set_led(self.connection, 0, 0)
        set_led(self.connection, 1, 0)

    def get_distances(self, ir):
        """
        Calculate the distance between robot and object directly ahead.
        Return -1 if no object detected.
        Return 0 if ir sensors at full.

        sensor = 136702 * exp(-0.47852 * distance)
        """
        ln_A = 11.83
        k = -0.48

        distances = (np.log(ir) - ln_A) / k

        for i,sensor in enumerate(ir):
            if sensor > 1000:
                distances[i] = 0
            elif sensor < 10:
                # far away (no reading)
                distances[i] = 1000

        return distances

    def spiral(self):
        logger.debug(self.spiral_count)
        if self.spiral_count < 300:
            turn(self.connection, 10, 2)
            self.spiral_count += 1
        elif self.spiral_count < 600:
            turn(self.connection, 2, 10)
            self.spiral_count += 1 
        else:
            stop(self.connection)
            self.spiral_count = 0

    def blink(self):
        self.stop()
        sleep = 0.25

        set_led(self.connection, 0, 1)
        time.sleep(sleep)
        for i in range(0,2):
            set_led(self.connection, 1, 2)
            set_led(self.connection, 0, 2)
            time.sleep(sleep)

        set_led(self.connection, 1, 0)
        set_led(self.connection, 0, 0)

class DoneTurning(Exception):
    pass

if __name__ == '__main__':
    robot = Kheperam()    
    robot.run()
