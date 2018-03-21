#!/usr/bin/env python2.7

import rospy
import cv2
import numpy
import math
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
import os


class Searcher:
	def __init__(self):
		self.t = Twist()
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		rospy.Subscriber('/turtlebot/scan', LaserScan, self.range)
		rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.looker)
		rospy.Subscriber('/turtlebot/turtlebot/events/bumper', BumperEvent, self.bumped)
		self.k_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=6)
		self.ori = 1
		self.found = [False, False, False, False]
		self.dist = 2.0
		self.bump = False
		self.flist = ''

	def bumped(self, msg):
		self.bump = bool(msg.state)

	def range(self, msg):
		# angle increment =  0.00163668883033
		# 1 degree in radians = 0.0174533
		# for a view of 21 degrees left or right we need ~222 steps of 0.00163668883033
		# this covers the middle 75% of the screen
		# angle min = -0.521567881107
		# angle max = 0.524276316166
		vp = 222
		avg = 0
		mid = (len(msg.ranges)/2) - 1
		for i in range(vp):

			if not math.isnan(msg.ranges[mid+i]):
				avg += msg.ranges[mid + i]

			if not math.isnan(msg.ranges[mid-i]):
				avg += msg.ranges[mid-i]

		avg /= vp*2
		self.dist = avg

	def	looker(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		self.display(image)
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_green = numpy.array([45, 100, 100])
		upper_green = numpy.array([75, 255, 255])
		lower_yellow = numpy.array([25, 100, 100])
		upper_yellow = numpy.array([35, 255, 255])
		lower_red1 = numpy.array([0, 100, 100])
		upper_red1 = numpy.array([8, 255, 255])
		lower_red2 = numpy.array([169, 100, 50])
		upper_red2 = numpy.array([170, 255, 255])
		lower_blue = numpy.array([115, 100, 100])
		upper_blue = numpy.array([135, 255, 255])

		maskG = cv2.inRange(hsv, lower_green, upper_green)
		maskY = cv2.inRange(hsv, lower_yellow, upper_yellow)
		maskR = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
		maskB = cv2.inRange(hsv, lower_blue, upper_blue)
		mask = [[maskG, "green", 0], [maskY, "yellow", 1], [maskR, "red", 2], [maskB, "blue", 3]]

		colour = list()

		for f in range(len(self.found)):
			if not self.found[f]:
				colour.append(mask[f])

		data = self.colour_choice(colour)
		if not data[0]:
			os.system('clear')
			self.display(image)
			if self.dist < 0.7:
				os.system('clear')
				print "avoiding @: " + str(self.dist)
				self.avoid()
			if self.bump:
				os.system('clear')
				print "collision, reversing"
				self.avoid()
			else:
				print "searching..."
				print self.flist + "has been found so far"
				print self.dist
				self.roaming()

		else:
			if not self.bump:
				self.mover(data[1], image)
			else:
				self.found[data[3]] = True
				self.flist = self.flist + data[2] + ", "
			self.display(image)

	def colour_choice(self, colour):

		got = False
		temp = 0
		tempcm = colour[0][0]
		tempname = colour[0][1]
		tempcol = colour[0][2]

		for cm in colour:
			M = cv2.moments(cm[0])

			if M['m00'] > 80000:
				if temp < M['m00']:
					temp = M['m00']
					tempcm = cm[0]
					tempname = cm[1]
					tempcol = cm[2]
					os.system('clear')
					print "got em.\ncolour: " + cm[1] + " m: " + str(M['m00'])
					print self.dist
				got = True

		reply = list()
		reply.append(got)
		reply.append(tempcm)
		reply.append(tempname)
		reply.append(tempcol)

		return reply
		
	def roaming(self):
		if self.dist > 2:
			speed = 1
		else:
			speed = 0.5
		self.t.linear.x = speed
		self.k_pub.publish(self.t)
		rospy.sleep(0.5)
		self.t.linear.x = 0.0
		self.t.angular.z = speed * 2 * self.ori
		self.k_pub.publish(self.t)
		self.ori *= -1
		rospy.sleep(0.5)

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
			self.t.angular.z = -float(err) / 400
			self.k_pub.publish(self.t)
			# END CONTROL

	def display(self, image):
		cv2.imshow("window", image)
		cv2.waitKey(3)

	def avoid(self):
		if self.dist < 0.5:
			speed = -1
		else:
			speed = -0.8

		self.t.linear.x = speed
		self.t.angular.z = speed * self.ori
		self.k_pub.publish(self.t)
		rospy.sleep(1)
		self.t.linear.x = 0.0
		self.t.angular.z = speed * 2 * self.ori
		self.k_pub.publish(self.t)



rospy.init_node('hunter', anonymous=True)
rospy.Rate(100)
Do = Searcher()
rospy.spin()
