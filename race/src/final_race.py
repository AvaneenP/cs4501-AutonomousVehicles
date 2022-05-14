#!/usr/bin/env python
import math
import rospy
from os.path import expanduser
from race.msg import pid_input
import tf 
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped 

global fov
global car_width
global lookahead
global waypoints
global scans
global obstacle_detection 

obstacle_detection = 0.6
scans = []

command_pub = rospy.Publisher('/car_5/offboard/command', AckermannDrive, queue_size = 1)

#### NOTES ######
# Early in the scans list is RIGHT
# Negative is RIGHT in scans
# Positive is LEFT in scans 
# angle_min is on the RIGHT in scans

# Return the angle for an index 
def getAngleFromIndex(angle_inc, index, fov):
    return (fov / -2.0) + (float(index) * math.degrees(angle_inc))

# Return the ranges for a given FOV
def getRangesForFOV(scans, fov):
    if not scans: return []
    lower_index = int(((fov / -2.0) - math.degrees(scans.angle_min)) / math.degrees(scans.angle_increment))
    higher_index = int((math.degrees(scans.angle_max) - math.degrees(scans.angle_min) - (math.degrees(scans.angle_max) - (fov / 2.0))) / math.degrees(scans.angle_increment))
    return [10.0 if math.isnan(d) else d for d in scans.ranges[lower_index : higher_index]]

# Return the range at a given angle 
def getRangeForAngle(angle, data):
    if not scans: return 0.0 
    index = int(((angle - math.degrees(scans.angle_min)) / math.degrees(scans.angle_increment)))
    return data.ranges[index] if not math.isnan(data.ranges[index]) else 5.0

# Piece-wise speed function based on distance 
def getVelocityForSteeringAngle(angle):
    # TODO
    if abs(angle) >= 15.0:
        return 30.0
    return 57.0

    # BEST ARGS: turns 30.0, straight 57.0

def getGoalWaypoint(data):
    x = data.pose.position.x
    y = data.pose.position.y
    dist = [math.sqrt(math.pow(wpt[0] - x, 2) + math.pow(wpt[1] - y, 2)) for wpt in waypoints]
    closest = dist.index(min(dist))
    best = closest 
    for i in range(int(len(dist) / 5)):
        check = (closest + i) % len(dist)
        if dist[check] <= lookahead and dist[check] >= dist[best]:
            best = check 
    best += 1
    print("Waypoint: " + str(waypoints[best][0]) + ", " + str(waypoints[best][1]))
    return best

def getAngleToGoal(data, wpt):
    x = data.pose.position.x
    y = data.pose.position.y 
    theta = math.degrees(tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                        data.pose.orientation.y,
                                                        data.pose.orientation.z,
                                                        data.pose.orientation.w))[2])
    gamma = math.degrees(math.atan(abs((wpt[1] - y) / (wpt[0] - x))))
    # Put both gamma and theta on a scale from 0 to 360
    print("Theta/gamma: " + str(theta) + "/" + str(gamma))
    if x > wpt[0] and y < wpt[1]: gamma = 180 - gamma 
    if x > wpt[0] and y > wpt[1]: gamma += 180
    if x < wpt[0] and y > wpt[1]: gamma = 360 - gamma 
    if theta < 0: theta += 360 
    # Decide whether turning left or right is faster
    alpha = theta - gamma 
    if alpha <= -180: alpha += 360
    if alpha >= 180: alpha -= 360 
    return -alpha * 1.2
    
    # dist = math.sqrt(math.pow(wpt[0] - x, 2) + math.pow(wpt[1] - y, 2))
    # curvature = (2.0*(abs(wpt[0]) - abs(x))/(math.pow(dist, 2)))
    # steering = math.atan(curvature * 0.33)
    # px = dist * math.cos(heading) + x
    # py = dist * math.sin(heading) + y
    # side = (wpt[0] - x)*(py - y) - (wpt[1] - y)*(px - x)
    # if side > 0: # Goal is to the right 
    #     steering *= 1.0
    # elif side < 0: # Goal is to the left 
    #     steering *= -1.0
    # return steering 

def getSteeringAngle(data, wpt):
    steering = getAngleToGoal(data, wpt)
    print("Steering: " + str(steering))
    # for angle in range(-5, 5):
    #     obstacle_range = getRangeForAngle(steering + angle, scans)
    #     if obstacle_range <= lookahead + obstacle_detection: # Avoid obstacle 
    #         steering_angle = findGap(steering + angle)
    #         print("Avoid obstacle: " + str(steering_angle))
    #         return steering_angle
    return steering

# Pick a gap to follow 
def findGap(close_angle):

    # Subset the scans to get data for only the desired FOV
    if not scans: return 0.0 
    ranges = getRangesForFOV(scans, fov)
    index = int((math.degrees(close_angle) - math.degrees(scans.angle_min)) / math.degrees(scans.angle_increment))
    print(str(len(ranges)) + " <> " + str(index))
        
    # Pick the largest gap and choose a direction
    target_angle = 0.0
    
    left_size = 0
    left_gap = -1
    right_size = 0
    right_gap = -1
    c = 0
    for i in range(0, min(index, len(ranges))):
        if ranges[i] >= lookahead + obstacle_detection: c += 1
        else: c = 0
        if c >= left_size: 
            left_size = c 
            left_gap = i - c 
            
    c = 0
    for i in range(min(index, len(ranges)-1), len(ranges)):
        if ranges[i] >= lookahead + obstacle_detection: c += 1
        else: c = 0
        if c >= right_size: 
            right_size = c 
            right_gap = i - c
        
    if right_size > left_size:
        target_angle = getAngleFromIndex(scans.angle_increment, right_gap + int(right_size / 2.0), fov)
    else: 
        target_angle = getAngleFromIndex(scans.angle_increment, left_gap + int(left_size / 2.0), fov)
    
    # Return an angle to turn towards (0 is straight ahead)
    return target_angle  

# Pick a gap to follow 
def path(odom_data):
    
    goal_wpt = getGoalWaypoint(odom_data)
    
    target_angle = getSteeringAngle(odom_data, waypoints[goal_wpt])

    speed = getVelocityForSteeringAngle(target_angle)
    
    # Return an angle to turn towards (0 is straight ahead)
    angle_mult = 1.0
    return target_angle * angle_mult, speed 

def callback_scan(data):
    global scans 
    scans = data 

# Handle the scans from the lidar
def callback_odom(data):
    global fov 
    global car_width
    global lookahead
    global waypoints 
    global scans 
    
    angle, speed = path(data)
    command = AckermannDrive()
    command.steering_angle = max(-100.0, min(100.0, angle))
    command.speed = max(0.0, min(100.0, speed)) 
    print("Angle: " + str(angle) + ", Speed: " + str(speed))
    command_pub.publish(command)

if __name__ == '__main__':
    fov = 180
    car_width = 0.5
    lookahead = 0.7
    home = expanduser('~')
    file = open(home+'/less_waypoints2_old.csv', 'r')
    # Each waypoint is x,y,heading,speed
    waypoints = [pnt.split(',') for pnt in file.readlines()]
    waypoints = [(float(p[0].strip()), float(p[1].strip())) for p in waypoints]
    print(waypoints[0])
    file.close()
    rospy.init_node('final_race', anonymous = True)
    rospy.Subscriber("/car_5/particle_filter/viz/inferred_pose", PoseStamped, callback_odom)
    rospy.Subscriber("/car_5/scan", LaserScan, callback_scan)
    rospy.spin()
