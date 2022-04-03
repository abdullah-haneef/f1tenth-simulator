#!/usr/bin/env python3

import math
import rospy
from fssim_nav.msg import pid_input
from ackermann_msgs.msg import AckermannDriveStamped

# PID Control Params
pid = 0.00
kp = 1.0 #TODO
kd = 0.75 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
vel_input = 2

# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward. 
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.

angle = 0.0
# Publisher for moving the car. 
# TODO: Use the coorect topic /car_x/offboard/command.
command_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 1)

def control(data):
	global pid
	global prev_error
	global vel_input
	global kp
	global kd
	global angle
	
	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller

	error = data.pid_error
	
	# 1. Scale the error
	# 2. Apply the PID equation on error  to compute steering
	
	print((float(error) - float(prev_error)))
	pid = (float(kp) * float(error)) + (float(kd) * (float(error) - float(prev_error)))  
	
	prev_error = error
	
	angle = -pid
	
	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDriveStamped()

	# TODO: Make sure the steering value is within bounds [-100,100]
	if command.drive.steering_angle <= 100 and angle >= -100: 
		command.drive.steering_angle = angle
		
	# TODO: Make sure the velocity is within bounds [0,100]
	command.drive.speed = float(vel_input)
	
	command_pub.publish(command)
	
if __name__ == '__main__':
	
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
