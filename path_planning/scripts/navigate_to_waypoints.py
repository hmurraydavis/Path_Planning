#!/usr/bin/env python
# subscribes to /particle node and publishes geometry/Twist msgs from /robot_direct to /cmd_vel node.
# publishes geometry/Twist msgs to navigate towards waypoints
# Path Planning Team 
# Computational Robotics 2014 Olin College

import tf
import math
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped

angle = 0
dist_target = 0
index = 0
speed = 0

list_of_waypoints = [(1,0)]

def transform_quaternion_to_angle(msg):
    """ Processes data from type Quaternion and returns current direction of robot in degrees from range 0 to 360 degrees"""
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w

    quaternion = (qx,qy,qz,qw)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    quaternion_angle = yaw*180/math.pi 
    return quaternion_angle

def calculate_heading(msg,index):
    """ Processes data from current position and returns heading of robot in degrees from range 0 to 360 degrees"""

    x_pos = msg.pose.position.x
    y_pos = msg.pose.position.y

    delta_y = list_of_waypoints[index][1] - y_pos
    delta_x = list_of_waypoints[index][0] - x_pos

    angle_target = math.atan2(delta_y,delta_x)*180/math.pi   

    return angle_target

def calculate_heading_speed(msg):
    """ Subscribes to /particle node, which publishes the mean of all particles produced by the robot particle filter.
    Processes data from /particle node of type PoseStamped, which gives an estimated current position and direction. Outputs constants
    speed and angle, which are parameters used in navigate_to_waypoint().
    """

    global index
    global angle
    #global dist_target
    global list_of_waypoints
    global speed

    x_pos = msg.pose.position.x
    y_pos = msg.pose.position.y

    angle_current = transform_quaternion_to_angle(msg)

    if index >= len(list_of_waypoints): #stop when last waypoint is completed
        speed = 0
        angle = 0

    else:
        speed = .05
        angle_target = calculate_heading(msg,index)

        if angle_target < 0:
            angle_target = 360 + angle_target

        if angle_current < 0:
            angle_current = 360 + angle_current

        angle = angle_target - angle_current

        if 0 <= angle_target <= 180 and 180 <= angle_current <= 360:
            angle = angle_target + 360 - angle_current

        if 0 <= angle_current <= 180 and 180 <= angle_target <= 360:
            angle = 360 - angle_current - angle_target

        dist_target = math.sqrt((list_of_waypoints[index][0]-x_pos) ** 2 + (list_of_waypoints[index][1]-y_pos) ** 2)

        if dist_target < 0.5:
            index += 1

    return index

def navigate_to_waypoint(pub):
    """ publishes to /cmd_vel node of type geometry_msgs/Twist"""
    r = rospy.Rate(10)
    while not (rospy.is_shutdown()):
        velocity_msg = Twist(Vector3(speed,0.0,0.0),Vector3(0.0,0.0,0.003*angle))
        pub.publish(velocity_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_direct', anonymous=True)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber('particle', PoseStamped, calculate_heading_speed)
        #sub1 = rospy.Subscriber('')
        navigate_to_waypoint(pub)
    except rospy.ROSInterruptException: pass
