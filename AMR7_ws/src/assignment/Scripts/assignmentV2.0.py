#!/usr/bin/env python2.7

import rospy
import cv2
import numpy
import sys
import math
import time
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from random import randint
import os

class Searcher:
	def __init__(self):
		self.t = Twist()
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		rospy.Subscriber('/turtlebot/scan', LaserScan, self.range)
		rospy.Subscriber('/turtlebot/odom', Odometry, self.compass)
		rospy.Subscriber('/turtlebot/turtlebot/events/bumper', BumperEvent, self.bumped)
		rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.looker)
		self.k_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=100, latch=True)
		self.found = [False, False, False, False]
		self.distm = 2.0
		self.distl = 2.0
		self.distr = 2.0
		self.bump = False
		self.flist = ''
		self.cBearing = 0
		self.oBearing = 0
		self.cpos = 0
		self.opos = 0
		self.data = list
		self.dlim = 1
		self.errcount = 0


	def compass(self, msg):
		self.cBearing = math.degrees(msg.pose.pose.orientation.z)
		self.cpos = msg.pose.pose.position.x

	def bumped(self, msg):
		self.bump = bool(msg.state)

	def range(self, msg):
		# angle increment =  0.00163668883033
		# 1 degree in radians = 0.0174533
		# for a view of 14 degrees left or right we need ~148 steps of 0.00163668883033
		# this covers the middle ~50% of the screen
		# for the sides two sections of 7 degrees will be used
		# angle min = -0.521567881107
		# angle max = 0.524276316166

		vp = 148
		avg = 0
		avl = 0
		avr = 0
		mid = (len(msg.ranges)/2) - 1
		left = len(msg.ranges) - 1

		for i in range(vp/2):
			if not math.isnan(msg.ranges[i]):
				avr += msg.ranges[i]

			if not math.isnan(msg.ranges[left-i]):
				avl += msg.ranges[left-i]

		for i in range(vp):
			if not math.isnan(msg.ranges[mid+i]):
				avg += msg.ranges[mid + i]

			if not math.isnan(msg.ranges[mid-i]):
				avg += msg.ranges[mid-i]

		avg /= vp * 2
		avl /= vp
		avr /= vp
		self.distl = avl
		self.distm = avg
		self.distr = avr

	def looker(self, msg):
		self.cpos = self.opos
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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

		if len(colour) > 0:
			self.data = self.colour_choice(colour)

		else:
			os.system('clear')
			rospy.signal_shutdown()

		if self.data[0]:
			self.display(image)
			self.mover(self.data[1], image)
			return

		if self.errcount > 10:
			self.Mayday()
			return

		else:
			self.display(image)
			os.system('clear')
			print "Roaming...\n"
			print "l: " + str(self.distl)
			print "m: " + str(self.distm)
			print "r: " + str(self.distr)
			print self.flist + "has been found so far"

			err = self.distl - self.distr

			if self.bump or self.distm < 0.6 or (self.distl < 0.6 and self.distr < 0.6):
				self.errcount += 1
				print "adjusting"
				self.adjust()
				self.reverse()
				return

			if err < -0.5:
				print "turning right"
				if self.distl < 1:
					self.t.angular.z = -1
					#self.t.linear.x = -0.1
					self.k_pub.publish(self.t)
				else:
					self.t.angular.z = -0.5
					self.t.linear.x = 0.6
					self.k_pub.publish(self.t)
				return

			if err > 0.5:
				print "turning left"
				if self.distr < 1:
					self.t.angular.z = 1
					#self.t.linear.x = -0.1
					self.k_pub.publish(self.t)
				else:
					self.t.angular.z = 0.5
					self.t.linear.x = 0.6
					self.k_pub.publish(self.t)
				return

			else:
				print "heading straight with error of: " + str(err)
				self.t.angular.z = 0.0
				self.t.linear.x = 1
				self.k_pub.publish(self.t)
				return


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
					print "l: " + str(self.distl)
					print "m: " + str(self.distm)
					print "r: " + str(self.distr)
				got = True

		reply = list()
		reply.append(got)
		reply.append(tempcm)
		reply.append(tempname)
		reply.append(tempcol)

		return reply

	def mover(self, mask, image):
		h, w, d = image.shape
		M = cv2.moments(mask)

		if M['m00'] > 0:
			cx = int(M['m10'] / M['m00'])
			cy = int(M['m01'] / M['m00'])
			cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
			# BEGIN CONTROL
			err = cx - w / 2

			if self.distr < 0.7:
				print "correcting angle to left"
				self.stop()
				self.t.angular.z = 0.5
				self.t.linear.x = -0.2
				self.k_pub.publish(self.t)
				rospy.sleep(0.5)
				self.t.angular.z = -0.5
				self.t.linear.x = -0.2
				self.k_pub.publish(self.t)
				return

			if self.distl < 0.7:
				print "correcting angle to right"
				self.stop()
				self.t.angular.z = -0.5
				self.t.linear.x = -0.2
				self.k_pub.publish(self.t)
				rospy.sleep(0.5)
				self.t.angular.z = 0.5
				self.t.linear.x = -0.2
				self.k_pub.publish(self.t)
				return

			if self.bump:
				self.stop()
				self.t.angular.z = float(err) / 400
				self.t.linear.x = -0.2
				self.k_pub.publish(self.t)
				rospy.sleep(0.5)
				return

			if self.distm < 0.9:
				self.errcount = 0
				self.stop()
				self.found[self.data[3]] = True
				self.flist = self.flist + self.data[2] + ", "
				self.t.linear.x = -1
				self.k_pub.publish(self.t)
				rospy.sleep(1)
				self.t.angular.z = 0.5
				self.k_pub.publish(self.t)
				rospy.sleep(1)
				return

			else:
				self.t.linear.x = 1
				self.t.angular.z = -float(err) / 200
				self.k_pub.publish(self.t)
			# END CONTROL


	def Mayday(self):
		os.system('clear')
		print "MAYDAY, PERFORMING, EMERGENCY MANEUVER"
		err = self.distl - self.distr
		if err < -1:
			self.t.angular.z = -0.5
			self.t.linear.x = -1
			self.k_pub.publish(self.t)
			self.errcount -= 2
		if err > 1:
			self.t.angular.z = 0.5
			self.t.linear.x = -1
			self.k_pub.publish(self.t)
			self.errcount -= 2
		else:
			self.t.angular.z = 0.0
			self.t.linear.x = -1
			self.k_pub.publish(self.t)
			self.errcount -= 3
		rospy.sleep(0.5)
		self.t.angular.z = 0.0
		self.t.linear.x = -1
		self.k_pub.publish(self.t)
		self.errcount -= 1
		return

	def display(self, image):
		cv2.imshow("window", image)
		cv2.waitKey(3)

	def stop(self):
		print "braking"
		self.t.linear.x = 0.0
		self.t.angular.z = 0.0
		self.k_pub.publish(self.t)

	def reverse(self):
		err = self.distl - self.distr

		dire = [1, -1]
		if err < -1:
			self.t.angular.z = -0.5
			self.t.linear.x = -0.5
			self.k_pub.publish(self.t)
		if err > 1:
			self.t.angular.z = 0.5
			self.t.linear.x = -0.5
			self.k_pub.publish(self.t)
		else:
			self.t.angular.z = dire[randint(0, 1)]
			self.t.linear.x = -0.5
			self.k_pub.publish(self.t)
		rospy.sleep(0.5)

	def adjust(self):
		err = self.distl - self.distr
		dire = [1, -1]

		if self.bump or self.distm < 0.7:
			if err < -0.5:
				self.t.angular.z = -1
				self.k_pub.publish(self.t)
			if err > 0.5:
				self.t.angular.z = 1
				self.k_pub.publish(self.t)
			else:
				self.t.angular.z = dire[randint(0, 1)]
				self.k_pub.publish(self.t)
		rospy.sleep(0.5)


def myhook():
  print "I found all within my reach, FEAR ME!!!!!!!!!!"

rospy.init_node('hunter', anonymous=True)
rospy.Rate(100)
start = time.time()
Do = Searcher()
rospy.spin()
rospy.on_shutdown(myhook)
end = time.time()
print(end - start)
