################################################################################
# MODULES IMPORTS
################################################################################

import cflib.crtp
import logging
import time
import sys

import math as math
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from enum import Enum

################################################################################
# GLOBAL VARIABLES
################################################################################

# Unsafe because many functions read and modify its value all the time
# You have no guarantee on the actual current value
my_unsafe_logging_variable = None

# Retrieve the URI from the client
URI = 'radio://0/80/2M'

################################################################################
# CALLBACKS
################################################################################

# This is called when an error occurs when reading sensor from the drone
def my_log_error_callback(logconf, msg):
	"""Callback from the log API when an error occurs"""

	print('Error when logging %s: %s' % (logconf.name, msg))

# This function is called periodically
# Data contains information as described below
def my_log_data_callback(timestamp, data, logconf):
	"""Callback from the log API when data arrives"""

	global my_unsafe_logging_variable
	my_unsafe_logging_variable=data

################################################################################
# DEFINITIONS
################################################################################

def coordinates():
	x_destination = 1.5
	y_destination = 1.5
	if counter==0:
		x = x_destination - my_unsafe_logging_variable['stateEstimate.x']
		y = y_destination - my_unsafe_logging_variable['stateEstimate.y']
		coordinates = [x,y]
		return coordinates
	elif counter==1:
		x_destination = 0 - x_destination
		y_destination = 0 - y_destination
		x = x_destination - my_unsafe_logging_variable['stateEstimate.x']
		y = y_destination - my_unsafe_logging_variable['stateEstimate.y']
		coordinates = [x,y]
		return coordinates

def angle_for_alignment(angle):
	test_angle = 360 - angle
	return test_angle

def angle(x,y):

	angle = abs(math.degrees(math.atan(x/y)))
	if x > 0 and y > 0 :
		print(90 - angle)
		angle = 90 - angle
	if x > 0 and y < 0 :
		print(270 + angle)
		angle = 270 + angle
	if x < 0 and y > 0 :
		print(90 + angle)
		angle = 90 + angle
	if x < 0 and y < 0 :
		print (270 - angle)
		angle = 270 - angle
	return angle

def turn_left_target(angle):
	if angle < 180 :
		mc.turn_left(angle)
	if angle > 180 :
		mc.turn_right(360-angle)

def distance_to_target(x,y):
	distance = abs(math.sqrt(x**2+y**2))
	return distance

def move_distance_to_target():
	time.sleep(0.1)
	right_coords = coordinates()
	print(right_coords)
	remaining_dist = distance_to_target(right_coords[0], right_coords[1])
	angle_to_cover = angle(right_coords[0], right_coords[1])
	turn_left_target(angle_to_cover)
	reorientation_ang = angle_for_alignment(angle_to_cover)
	time.sleep(0.1)
	while remaining_dist > 0.20:
		time.sleep(0.1)
		angle_to_cover = angle_to_cover
		reorientation_ang = reorientation_ang
		mc._set_vel_setpoint(0.2,0,0,0)
		current_obstacle = obstacle_detection()
		LED(remaining_dist)
		if current_obstacle != 1:
			print("I see an obstacle")
			mc._set_vel_setpoint(0,0,0,0)
			obstacle_avoidance(obstacle_detection())
			time.sleep(0.3)
			turn_left_target(reorientation_ang)
			time.sleep(0.3)
			print("Reorientation angle", reorientation_ang)
			right_coords = coordinates()
			print(right_coords)
			angle_to_cover = angle(right_coords[0],right_coords[1])
			print("Angle to cover", angle_to_cover)
			reorientation_ang = angle_for_alignment(angle_to_cover)
			mc._set_vel_setpoint(0,0,0,0)
			turn_left_target(angle_to_cover)
			remaining_dist = distance_to_target(right_coords[0],right_coords[1])
		right_coords = coordinates()
		remaining_dist = distance_to_target(right_coords[0],right_coords[1])
		print (remaining_dist)

	time.sleep(0.1)
	turn_left_target(reorientation_ang)
	mc._set_vel_setpoint(0,0,0,0)
	LED(remaining_dist)
	mc.land(velocity=0.1)
	print("The drone has landed")

def is_close(range):

	MIN_DISTANCE = 0.3

	if range is None:
		return False
	else:
		return range < MIN_DISTANCE

def buzzer(obstacle):
	if obstacle == 1:
		cf.param.set_value('PROTO_LSM.IO1_DUTY','{:d}'.format(0))
		cf.param.set_value('PROTO_LSM.IO1_FRQDIV','{:d}'.format(0))
		cf.param.set_value('PROTO_LSM.IO1_ON','{:d}'.format(0))
		time.sleep(0.1)
	else :
		cf.param.set_value('PROTO_LSM.IO1_DUTY','{:d}'.format(1))
		cf.param.set_value('PROTO_LSM.IO1_FRQDIV','{:d}'.format(2))
		cf.param.set_value('PROTO_LSM.IO1_ON','{:d}'.format(1))
		time.sleep(0.1)

def LED(remaining_dist):
	if remaining_dist > 0.3:
		cf.param.set_value('PROTO_LSM.IO4_DUTY','{:d}'.format(500))
		cf.param.set_value('PROTO_LSM.IO4_FRQDIV','{:d}'.format(1000))
		cf.param.set_value('PROTO_LSM.IO4_ON','{:d}'.format(1))
		time.sleep(0.1)
	else :
		cf.param.set_value('PROTO_LSM.IO4_DUTY','{:d}'.format(0))
		cf.param.set_value('PROTO_LSM.IO4_FRQDIV','{:d}'.format(0))
		cf.param.set_value('PROTO_LSM.IO4_ON','{:d}'.format(0))
		time.sleep(0.1)

def obstacle_detection():
	obstacle = 1
	if is_close(multiranger.front):
		obstacle = 2
		print("Can't move, obstacle is front")
	elif is_close(multiranger.left):
		obstacle = 3
		print("Can't move, obstacle is left")
	elif is_close(multiranger.right):
		obstacle = 4
		print("Can't move, obstacle is right")
	return obstacle

def obstacle_avoidance(obstacle):

	current_obstacle = obstacle
	overcoming = 1

	while current_obstacle == 2:
		time.sleep(0.01)
		mc._set_vel_setpoint(0,0.2,0,0)
		buzzer(current_obstacle)
		current_obstacle = obstacle_detection()
		overcoming = 1
	if overcoming == 1:
		mc.move_distance(0,0.3,0,velocity=0.1)
		mc.forward(0.2)

	mc.stop()

	if current_obstacle == 3:
		time.sleep(0.01)
		print("Left obstacle")
		buzzer(current_obstacle)
		mc.move_distance(0,-0.2,0,velocity=0.1)
		current_obstacle=obstacle_detection()

	if obstacle == 4:
		time.sleep(0.01)
		print("Right obstacle")
		buzzer(current_obstacle)
		mc.move_distance(0,0.2,0,velocity=0.1)
		current_obstacle=obstacle_detection()

	buzzer(current_obstacle)

################################################################################
# MAIN
################################################################################

if __name__ == '__main__':
	# Initialize the low-level drivers (don't list the debug drivers)
	# This just has to be done every time...
	cflib.crtp.init_drivers(enable_debug_driver=False)

	# Let's now create the objects that actually get sensors values from
	# the drone
	# Watch the period [ms]! -> this is the rate at which the callback above
	# will be called
	my_log_config = LogConfig(name='TestLogConfig', period_in_ms=1000)

	# This is how you can add a new variable
	my_log_config.add_variable('stateEstimate.x', 'float')# Find name in client
	my_log_config.add_variable('stateEstimate.y', 'float')
	my_log_config.add_variable('stateEstimate.z', 'float')

	my_unsafe_logging_variable = {'stateEstimate.x': 0, 'stateEstimate.y': 0, 'stateEstimate.z': 0}

	# Initialize a Crazyflie object (i.e. your drone)
	cf = Crazyflie(rw_cache='./cache')
	with SyncCrazyflie(URI, cf=cf) as scf:
		with MotionCommander(scf) as mc:
			with Multiranger(scf) as multiranger:

				# Tell the drone to send you data from the sensor from above
				cf.log.add_config(my_log_config)

				# Add the data and error callbacks declared above
				my_log_config.data_received_cb.add_callback(my_log_data_callback)
				my_log_config.error_cb.add_callback(my_log_error_callback)

				# Start the logging
				my_log_config.start()

				# Starting here, my_unsafe_logging_variable contains the information
				# I want
				counter = 0

				time.sleep(0.5)
				move_distance_to_target()
				time.sleep(0.5)
				mc.take_off(velocity=0.3)
				counter = 1
				move_distance_to_target()

################################################################################
