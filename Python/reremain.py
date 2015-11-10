from remain import Kheperam, Point, DoneTurning
import math
import numpy as np
import logging
import sys
import time
from odometry_plot import OdometryPlot
from collections import namedtuple
from khepera_functions import *

STATES = {'AVOID': 0,'FOLLOW': 1,'EXPLORE': 2,'GO HOME': 3,'GO TO FOOD': 4}
ivd = {v: k for k, v in STATES.items()}

START_POINT = Point(x=500,y=0)

FOLLOW_SPEED = 2
FOOD_SPEED = 5

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.DEBUG)
logger.addHandler(ch)

class Robot(Kheperam):

	'''
	Inherits from previous attempt
	'''

	def __init__(self):

		super(self.__class__, self).__init__()

		self.state = STATES['GO TO FOOD']
		self.food_at = START_POINT
		self.target = self.food_at
		self.parent = None
		self.careful = 0
		self.wait = 0

	def update(self):

		self.update_vars()

		self.previous_state = self.state

		# Sense and change state to Avoid if needed
		self.sense()

		self.graph.update(self.x, self.y)

		if self.state == STATES['AVOID']:
			self.avoid()
		elif self.state == STATES['FOLLOW']:
			self.follow()
		elif self.state == STATES['EXPLORE']:
			self.explore()
		else:
			self.hunt()

		logger.info(ivd[self.state])

	def sense(self):

		if self.detect_ahead() and not self.turning_home:
			self.state = STATES['AVOID']

	def avoid(self):

		turn_v = 2

		if self.is_left():
			target = self.theta + ((5.0 * math.pi) / 3.0)
		else:
			target = self.theta + (math.pi / 3.0)

		error = (self.theta - target) % (2*math.pi)

		if error > math.pi:
			# turn left
			turn(self.connection,-turn_v,turn_v)
		else:
			# turn right
			turn(self.connection,turn_v,-turn_v)

		while error > (math.pi/16):

			self.update_vars()
			self.graph.update(self.x, self.y)

			error = (self.theta - target) % (2*math.pi)

		self.stop()

		if not self.previous_state == STATES['EXPLORE']:
			self.state = STATES['FOLLOW']
			self.parent = self.previous_state
			self.follow_start()
		else:
			self.state = STATES['EXPLORE']

	def follow_start(self):

		go(self.connection, FOLLOW_SPEED)

		if self.is_left():
			self.wall = 'left'
		else:
			self.wall = 'right'

	def follow(self):

		dir_target = 'left' if self.phi > math.pi else 'right'

		if self.wall == 'left':
			dist = sum(self.ir_sensors[0:2]) / 3.0
		else:
			dist = sum(self.ir_sensors[3:5]) / 3.0

		if dist < 5 or dir_target != self.wall:
			self.state = self.parent
			go(self.connection,self.EXPLORE_SPEED)
			self.wall = None
			self.wait = 5

	def hunt(self):

		distance_error = abs(self.target.x - self.x) + abs(self.target.y - self.y)
		if distance_error < 5 and self.state == STATES['GO HOME']:
			self.blink()
			self.has_food = False
			self.target = self.food_at
			self.state = STATES['GO TO FOOD']
			return
		elif distance_error < 5:
			self.state = STATES['EXPLORE']
			return
		if not self.wait:
			try:
				self.point_home()
			except DoneTurning:
				go(self.connection, FOOD_SPEED)
				self.careful = 10

		if self.wait > 0:
			self.wait = self.wait - 1


	def is_left(self):
		return sum(self.distances[0:2]) < sum(self.distances[3:5])

	def key_pressed(self, event):
		self.state = STATES['GO HOME']
		self.food_at = Point(x=self.x, y=self.y)
		self.target = self.home
		self.blink()

if __name__ == '__main__':
    robot = Robot()    
    robot.run()