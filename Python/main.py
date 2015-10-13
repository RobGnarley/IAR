# --------------------------------------
# Intelligent Autonomous Robotics
# --------------------------------------
#
# Finlay McAfee 	- s1220880
# Mattias Appelgren - s1202144
#
# Main control loop for Khepera II robot
# --------------------------------------

from khepera_functions import *

STATES = ['BEGIN', 'EXPLORE', 'AVOID', 'FOLLOW_WALL']
EXPLORE_SPEED = 4
TURN_SPEED = 2
WALL_SPEED = 2
IDEAL_IR = 100
K1 = 0.0005
K2 = 0.9

def run():

	''' main control loop '''

	s = open_connection()
	set_counts(s, 0, 0)
	current_state = STATES[0]
	turning = None
	wall = None
	error = 0
	v_left = WALL_SPEED
	v_right = WALL_SPEED

	while True:

		ir_sensors = read_IR(s)

		print current_state

		# BEGIN
		if current_state == STATES[0]:

			
			go(s,EXPLORE_SPEED)

			# Explore
			current_state = STATES[1]

		# EXPLORE
		elif current_state == STATES[1]:

			# Move forward
			go(s,EXPLORE_SPEED)

			if is_object_ahead(ir_sensors):
				stop(s)
				# Avoid obstacle
				current_state = STATES[2]

			elif ir_sensors[0] > 120:
				# Follow wall on left
				wall = 'left'
				current_state = STATES[3]

			elif ir_sensors[5] > 120:
				# Follow wall on left
				wall = 'right'
				current_state = STATES[3]

		# AVOID
		elif current_state == STATES[2]:

			print "Avoiding"

			if not turning:
				if ir_sensors[2] > ir_sensors[3]:
					# Turn right
					turning = 'right'
					turn(s,TURN_SPEED,-TURN_SPEED)
				else:
					# Turn left
					turning = 'left'
					turn(s,-TURN_SPEED,TURN_SPEED)
			elif turning == 'right':
				if ir_sensors[0] < 100 and ir_sensors[2] < 70:
					# Follow wall on left
					wall = 'left'
					current_state = STATES[3]
					turning = None 
			else:
				# Otherwise turning left
				if ir_sensors[5] < 100 and ir_sensors[3] < 70:
					# Follow wall on right
					wall = 'right'
					current_state = STATES[3]
					turning = None

		# FOLLOW WALL
		elif current_state == STATES[3]:

			print "Following on " + wall

			# Only changing speed of left wheel
			# Hence the only change between the left
			# and right wheel is the sign of the velocity
			# and the sensor being read

			if wall == 'left':
				sign = -1
				sensor = ir_sensors[0]
			else:
				# else right
				sign = 1
				sensor = ir_sensors[5]

			old_error = error
			error = IDEAL_IR - sensor
			delta_error = error - old_error

			# Negative error - too close, drive away
			# Postive error  - too far away, drive closer
			
			delta_v_left = (K1 * error * sign) + (K2 * delta_error * sign)
			v_left = int(v_left + delta_v_left)

			if abs(v_left) > 15:
				v_left = 2

			turn(s,v_left,v_right)

			print 'v_left: ' + str(v_left)

			#if sensor < 65:
				# Lost wall, go back to exploring
				# current_state = STATES[1]

		time.sleep(0.001)

def is_object_ahead(ir_sensors):

	return ir_sensors[2] > 100 or ir_sensors[3] > 100


if __name__ == "__main__" :

	run()
