#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan


# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
# forward_projection = 0.8	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
# desired_distance = 0.7	# distance from the wall (in m). (defaults to right wall)
vel = 15 		# this vel variable is not really used here.
# error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
    for i in range(len(data.ranges)):
        dist = data.ranges[i]
        data.ranges[i] = dist if not math.isnan(dist) else 15.0
    return data.ranges

def callback(data):
	# global forward_projection

    listofRanges = getRange(data)

    for 


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('follow_the_gap',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_5/scan",LaserScan,callback)
	rospy.spin()
