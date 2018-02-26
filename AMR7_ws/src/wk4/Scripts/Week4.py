#!/usr/bin/env python

import rospy
import cv2
import numpy
import math
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

wheel_radius = 0.038
robot_radius = 0.1875
t = Twist()
# computing the forward kinematics for a differential drive
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_r - c_l) / (2 * robot_radius)
    return (v, a)


# computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
    c_l = v - (robot_radius * a) 
    c_r = v + (robot_radius * a) 
    w_l = c_l / wheel_radius
    w_r = c_r / wheel_radius
    return (w_l, w_r)


# inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(t):
    return inverse_kinematics(t.linear.x, t.angular.z)

def move(angle):
    t.angular.z = angle.data
    k_pub.publish(t)

(w_l, w_r) = inverse_kinematics(0.0, 1.0)
print "w_l = %f,\tw_r = %f" % (w_l, w_r)

(v, a) = forward_kinematics(w_l, w_r)
print "v = %f,\ta = %f" % (v, a)

rospy.init_node('mover', anonymous=True)
k_sub = rospy.Subscriber('/wheel_vel_left', Float32, move)
k_pub = rospy.Publisher('/turtlebot_2/cmd_vel', Twist, queue_size=100)
rospy.spin()
#(w_l, w_r) = inverse_kinematics_from_twist(t)
#print "w_l = %f,\tw_r = %f" % (w_l, w_r)
