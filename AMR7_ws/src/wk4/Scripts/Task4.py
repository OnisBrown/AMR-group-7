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

class Follower:
	def __init__(self):
		self.t = Twist()
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		cv2.namedWindow("mask", 1)
		self.image_sub = rospy.Subscriber('/turtlebot_2/camera/rgb/image_raw', Image, self.mover)
		self.k_pub = rospy.Publisher('/turtlebot_2/cmd_vel', Twist, queue_size=100)




	def mover(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_green = numpy.array([45, 100, 50])
		upper_green = numpy.array([75, 255, 255])
		mask = cv2.inRange(hsv, lower_green, upper_green)
		h, w, d = image.shape
		M = cv2.moments(mask)
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
			# BEGIN CONTROL
			err = cx - w/2
			self.t.linear.x = 0.1
			self.t.angular.z = -float(err) / 200
			self.k_pub.publish(self.t)
			# END CONTROL
		else:
			self.t.angular.z = 0.3
			self.k_pub.publish(self.t)
		
		cv2.imshow("window", image)
		cv2.imshow("mask", mask)
		cv2.waitKey(3)
rospy.init_node('mover', anonymous=True)
Do = Follower()
rospy.spin()
#(w_l, w_r) = inverse_kinematics_from_twist(t)
#print "w_l = %f,\tw_r = %f" % (w_l, w_r)
