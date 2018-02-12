#!/usr/bin/env python
import sys
import time
import rospy
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import Twist

rospy.init_node('robot_mover', anonymous=True)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
move = Twist()

move.linear.y = 0.0
move.linear.x = 0.1
move.angular.x = 0.0
move.angular.y = 1.0
move.angular.z = 0.4



while not rospy.is_shutdown():
    velocity_publisher.publish(move)