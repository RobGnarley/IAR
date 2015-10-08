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
K = 0.0001

def run():

	''' main control loop '''

	s = open_connection()
	set_counts(s, 0, 0)
	current_state = STATES[0]
	is_moving = False
	turning = None
	wall = None
	v_left = WALL_SPEED
	v_right = WALL_SPEED

	while True:

		ir_sensors = read_IR(s)

		if len(ir_sensors) < 8:
			print "WARNING: IR sensors returned " + str(ir_sensors)
			continue

		print current_state

		# BEGIN
		if current_state == STATES[0]:

			# Move Forward
			is_moving = go_safe(s,EXPLORE_SPEED,is_moving)

			# Explore
			current_state = STATES[1]

		# EXPLORE
		elif current_state == STATES[1]:

			# Move forward if not already
			is_moving = go_safe(s,EXPLORE_SPEED,is_moving)

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
			else:
				# Otherwise turning left
				if ir_sensors[5] < 100 and ir_sensors[3] < 70:
					# Follow wall on right
					wall = 'right'
					current_state = STATES[3]

		elif current_state == STATES[3]:

			stop(s)

			break

			# PROP CONTROL NOT FINISHED

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

			error = IDEAL_IR - sensor

			# Negative error - too close, drive away
			# Postive error  - too far away, drive closer
			
			v_left = int(v_left + (K * error * sign))

			if abs(v_left) > 15:
				v_left = 2

			turn(s,v_left,v_right)

			print 'v_left: ' + str(v_left)

			if sensor < 65:
				# Lost wall, go back to exploring
				current_state = STATES[1]

		time.sleep(0.001)


def go_safe(s, speed, is_moving):

	if is_moving:
		return is_moving
	else:
		go(s,speed)
		return True

def is_object_ahead(ir_sensors):

	return ir_sensors[2] > 100 or ir_sensors[3] > 100


if __name__ == "__main__" :

	run()
