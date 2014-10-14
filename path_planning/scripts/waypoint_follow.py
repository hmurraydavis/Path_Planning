#!/usr/bin/env python
# Path_Planning Team
# Jasper Chen

import re
import tf
import math
import rospy
from std_msgs.msg import String
import parse_map
from geometry_msgs.msg import Twist, Vector3, PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
#import parse_map

quaternion = -1000
angle = 0
index = 0
speed = 0

print 'hi'

def start():
    print 'hi'
    list_of_way_points = parse_map.get_list_of_waypoints()
    return list_of_way_points

way_points = start()

print way_points

list_of_way_points = [(1,1)]

def parse_waypoint_list(msg):
    m = re.findall('\((.*?)\)',msg)
    print m

def calculate_heading_speed(msg):
    """ Subscribes to /particle node, which is the mean of all particles produced by the particle filter.
    Processes data from /particle node of type PoseStamped, which gives an estimated current position and direction. Outputs constants
    speed and angle, which are parameters used in get_to_waypoint() and published to /cmd_vel node of type geometry_msgs/Twist.
    """

    global index
    global angle
    global list_of_way_points
    global speed

    x_pos =  msg.pose.position.x
    y_pos = msg.pose.position.y

    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw =  msg.pose.orientation.w

    quaternion = (qx,qy,qz,qw)

    euler = tf.transformations.euler_from_quaternion(quaternion)

    angle_current = euler[2]*180/math.pi

    if index >= len(list_of_way_points):
        angle = 0
        speed = 0

    else:
        speed = .1
        delta_y = list_of_way_points[index][1] - y_pos
        delta_x = list_of_way_points[index][0] - x_pos
        dist_target = math.sqrt((list_of_way_points[index][0]-x_pos) ** 2 + (list_of_way_points[index][1]-y_pos) ** 2)
        angle_target = math.atan2(delta_y,delta_x)*180/math.pi

        if angle_target<0:
            angle_target = 360 + angle_target

        if angle_current<0:
            angle_current = 360 + angle_current

        angle = angle_target - angle_current

        if 0 <= angle_target <= 180 and 180 <= angle_current <= 360:
            angle = angle_target + 360 - angle_current

        if 0 <= angle_current <= 180 and 180 <= angle_target <= 360:
            angle = 360 - angle_current - angle_target

        if dist_target < 0.5:
            index += 1

    return index

def get_to_waypoint(pub):
    r = rospy.Rate(10)
    while not (rospy.is_shutdown()):
        velocity_msg = Twist(Vector3(speed,0.0,0.0),Vector3(0.0,0.0,0.003*angle))
        pub.publish(velocity_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_direct', anonymous=True)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        sub1 = rospy.Subscriber('particle', PoseStamped, calculate_heading_speed)
        sub2 = rospy.Subscriber('waypoint_list', String, parse_waypoint_list)
        get_to_waypoint(pub)
    except rospy.ROSInterruptException: pass
