#!/usr/bin/env python
import sys
import time
import rospy
import roslib
from std_msgs.msg import String
from geometry_msgs.msg import Twist

rospy.init_node('robot_mover', anonymous=True)
velocity_publisher = rospy.Publisher('turtlebot_1/cmd_vel', Twist, queue_size=100)
move = Twist()

while not rospy.is_shutdown():
    timer = time.time() + 10 
    move.linear.y = 0.0
    move.linear.x = 0.3
    move.angular.x = 0.0
    move.angular.y = 0.0
    move.angular.z = 1.5
    moving = True
    while moving:
        if time.time() > timer:
            moving = False
        velocity_publisher.publish(move)
        
    time.sleep(2)
    
    moving = True
    timer = time.time() + 10
    move.angular.z = 0
    while moving:
        if time.time() > timer:
            moving = False
        move.linear.x = 0.6
        velocity_publisher.publish(move)
        time.sleep(0.3)
        move.linear.x = -0.6
        velocity_publisher.publish(move)
        time.sleep(0.3)
        
    time.sleep(2)
    
    moving = True
    move.linear.x = 0.0
    timer = time.time() + 10
    while moving:
        if time.time() > timer:
            moving = False
        
        move.angular.z = 1
        velocity_publisher.publish(move)
        time.sleep(0.3)
        move.angular.z = -1
        velocity_publisher.publish(move)
        time.sleep(0.3)
        