#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params
# kp = 0.0 #TODO
# kd = 0.0 #TODO
# ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.

 
# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward. 
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.

# Publisher for moving the car. 
# TODO: Use the correct topic /car_x/offboard/command.
command_pub = rospy.Publisher('/car_5/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global total_error 
	global vel_input
	global max_vel
	global kp
	global kd
	global ki 

	print("PID Control Node is listening to error")
	
	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller
	vel_input = data.pid_vel 
	error = data.pid_error 
	
	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering
	
	C = 100.0 # Scale
	e = kp * error 
	de = kd * (prev_error - error)
	inte = ki * total_error 
	angle = C * (e + de + inte)
	speed = round(max_vel - 20 * abs(error)) * 1.0

	prev_error = error 
	total_error += error 

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	command.steering_angle = max(-100.0, min(100.0, angle))

	# TODO: Make sure the velocity is within bounds [0,100]
	command.speed = max(0.0, min(40.0, speed)) 
	print(speed)

	# Move the car autonomously
	command_pub.publish(command)

if __name__ == '__main__':
	global kp
	global kd
	global ki
	global vel_input
	global max_vel
	global prev_error 
	global total_error 
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	ki = input("Enter Ki Value: ")
	vel_input = input("Enter desired velocity: ")
	max_vel = min(100, vel_input)
	prev_error = 0.0
	total_error = 0.0
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
