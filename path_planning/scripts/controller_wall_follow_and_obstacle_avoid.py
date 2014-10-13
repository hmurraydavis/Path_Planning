#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import tf
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3, PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion

quaternion = -1000
angle = 0
dist_target = 0
index = 0
speed = 0

list_of_points = [(1,1)]

def scan_received(msg):
    """ Processes data from particle"""
    global index
    global angle
    global dist_target
    global list_of_points
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

    if index >= len(list_of_points):
        angle = 0
        speed = 0
    else:
        speed = .05
        delta_y = list_of_points[index][1] - y_pos
        delta_x = list_of_points[index][0] - x_pos
        dist_target = math.sqrt((list_of_points[index][0]-x_pos)**2+(list_of_points[index][1]-y_pos)**2)
        angle_target = math.atan2(delta_y,delta_x)*180/math.pi
        if angle_target<0:
            angle_target = 360 + angle_target
        if angle_current<0:
            angle_current = 360 + angle_current
        angle = angle_target - angle_current
        if 0<= angle_target <= 180 and 180<=angle_current<= 360:
            angle = angle_target + 360 - angle_current
        if 0<= angle_current <= 180 and 180<=angle_target<= 360:
            angle = 360 - angle_current - angle_target

        if dist_target < 0.5:
            index += 1

    #print 'index'
   # print index


    #print dist_target
    #print 'tar'
    #print angle_target
   # print 'curr'
   # print angle_current



   # print 'ang'
   # print angle

    return dist_target, angle

 
def follow_waypoint(pub):
    r = rospy.Rate(10)
    while not (rospy.is_shutdown()):
        #print angle


        #print quaternion
        #euler = tf.transformations.euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]

        #print math.atan2(5,-5)*180/math.pi

        velocity_msg = Twist(Vector3(speed,0.0,0.0),Vector3(0.0,0.0,0.003*angle))
        pub.publish(velocity_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_direct', anonymous=True)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber('particle', PoseStamped, scan_received)
        follow_waypoint(pub)
    except rospy.ROSInterruptException: pass
