#!/usr/bin/env python2.7

import rospy
import cv2
import numpy
import math
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import os


class Searcher:
	def __init__(self):
		self.t = Twist()
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		cv2.namedWindow("mask", 1)
		rospy.Subscriber('/turtlebot/scan', LaserScan, self.range)
		rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.looker)
		self.k_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=100)
		self.ori = 1
		self.found = [False, False, False, False]
		self.dist = 30

	def range(self, msg):
		# angle increment =  0.00163668883033
		# 1 degree in radians = 0.0174533
		# for a view of 1 degrees left or right we need ~11 steps of 0.00163668883033
		# angle min = -0.521567881107
		# angle max = 0.524276316166
		avg = 0
		new = 0
		mid = (len(msg.ranges)/2) - 1
		for i in range(11):
			if math.isnan(msg.ranges[mid+i]):
				avg += 0
			else:
				avg += msg.ranges[mid+i]
				print avg

			if math.isnan(msg.ranges[mid-i]):
				avg += 0
			else:
				avg += msg.ranges[mid-i]
				print avg

		avg /= 22
		print str(avg)
		self.dist = avg

	def	looker(self, msg):
		if self.dist < 0.45:
			os.system('clear')
			print "avoiding @: " + str(self.dist)
			self.avoid()
			self.display(tempcm, image)
			return

		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_green = numpy.array([45, 100, 50])
		upper_green = numpy.array([75, 255, 255])
		lower_yellow = numpy.array([20, 100, 50])
		upper_yellow = numpy.array([35, 255, 255])
		lower_red1 = numpy.array([0, 100, 50])
		upper_red1 = numpy.array([8, 255, 255])
		lower_red2 = numpy.array([169, 100, 50])
		upper_red2 = numpy.array([170, 255, 255])
		lower_blue = numpy.array([110, 100, 50])
		upper_blue = numpy.array([130, 255, 255])

		maskG = cv2.inRange(hsv, lower_green, upper_green)
		maskY = cv2.inRange(hsv, lower_yellow, upper_yellow)
		maskR = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
		maskB = cv2.inRange(hsv, lower_blue, upper_blue)
		mask = [[maskG, "green"], [maskY, "yellow"], [maskR, "red"], [maskB, "blue"]]
		colour = list()

		for f in range(len(self.found)):
			if not self.found[f]:
				colour.append(mask[f])

		got = False
		temp = 0
		count = 0
		tempcm = colour[0]
		for cm in colour:
			M = cv2.moments(cm[0])

			if M['m00'] > 80000:
				if temp < M['m00']:
					temp = M['m00']
					tempcm = cm
					os.system('clear')
					print "got em colour: " + cm[1] + " m: " + str(M['m00'])
				got = True

			count += 1

		if not got:
			os.system('clear')
			print "searching..."
			self.roaming()

		else:

			self.mover(tempcm, image)
			if self.dist < 0.7:
				self.found[count] = False
				print 'removing colour' + colour[count][1]

		self.display(tempcm, image)
		
	def roaming(self):
		self.t.linear.x = 0.7
		self.k_pub.publish(self.t)
		rospy.sleep(1.5)
		self.t.linear.x = 0.0
		self.t.angular.z = 0.6 * self.ori
		self.k_pub.publish(self.t)
		rospy.sleep(1)
		self.ori *= -1

	def mover(self, mask, image):
		h, w, d = image.shape
		M = cv2.moments(mask)

		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
			# BEGIN CONTROL
			err = cx - w/2
			self.t.linear.x = 0.5
			self.t.angular.z = -float(err) / 200
			self.k_pub.publish(self.t)
			# END CONTROL

	def display(self, mask, image):
		cv2.imshow("window", image)
		cv2.imshow("mask", mask)
		cv2.waitKey(3)

	def avoid(self):
		self.t.linear.x = -0.3
		self.t.angular.z = 0.6 * self.ori
		self.k_pub.publish(self.t)
		#self. ori *= -1
		rospy.sleep(0.5)


rospy.init_node('hunter', anonymous=True)
rospy.Rate(100)
Do = Searcher()
rospy.spin()
