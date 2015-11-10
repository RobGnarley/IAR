from remain import Kheperam, Point
import math
import numpy as np
import logging
import sys
import time
from odometry_plot import OdometryPlot
from collections import namedtuple
from khepera_functions import *

STATES = {'AVOID': 0,'STUPID FOLLOW': 1,'EXPLORE': 2,'GO HOME': 3,'GO TO TARGET': 4}

START_POINT = Point(100,100)

class Robot(kheperam):

	'''
	Inherits from previous attempt
	'''

	def __init__(self):

		super(robot, self).__init__()

		self.state = STATES['GO HOME']
		self.food_at = START_POINT
		self.target = self.food_at

	def update(self):

		self.update_vars()

		self.previous_state = self.state

		# Sense and change states
		self.sense()

		self.graph.update(self.x, self.y)

		if self.state == STATES['AVOID']:
			self.avoid()
		elif self.state == STATES['EXPLORE']:
			self.explore()
		elif self.state == STATES['GO HOME']:
			self.hunt()
		elif self.state == STATES['GO TO TARGET']:
			self.hunt()
		else:
			self.explore()

	def avoid():

		turn_v = 2

		if sum(self.distances[0:2]) < sum(self.distances[3:5]):
			target = (5.0 * math.pi) / 3.0
		else:
			target = math.pi / 3.0

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
			self.state = 







