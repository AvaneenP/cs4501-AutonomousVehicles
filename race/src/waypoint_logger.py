#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from numpy import linalg as LA
from geometry_msgs.msg import PoseStamped 
import time 

home = expanduser('~')
file = open(home+'/waypoints.csv', 'w')

def save_waypoint(data):
    # quaternion = np.array([data.pose.pose.orientation.x, 
    #                        data.pose.pose.orientation.y, 
    #                        data.pose.pose.orientation.z, 
    #                        data.pose.pose.orientation.w])

    # euler = tf.transformations.euler_from_quaternion(quaternion)
    # speed = LA.norm(np.array([data.twist.twist.linear.x, 
    #                           data.twist.twist.linear.y, 
    #                           data.twist.twist.linear.z]),2)
    # if data.twist.twist.linear.x>0.:
    #     print(data.twist.twist.linear.x)

    file.write('%f, %f\n' % (data.pose.position.x,
                                     data.pose.position.y,
                                    #  euler[2],
                                    ))
    # print("Writing to csv")

def shutdown():
    file.close()
    print('Goodbye')
 
def listener():
    rospy.init_node('waypoints_logger', anonymous=True)
    rospy.Subscriber('/car_5/particle_filter/viz/inferred_pose', PoseStamped, save_waypoint)
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving waypoints...')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass