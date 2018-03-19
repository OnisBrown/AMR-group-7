import rospy
import cv2
import numpy
import math
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class Searcher:
	def __init__(self):
		self.t = Twist()
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		cv2.namedWindow("mask", 1)
		self.image_sub = rospy.Subscriber('/turtlebot_2/camera/rgb/image_raw', Image, self.mover)
		self.k_pub = rospy.Publisher('/turtlebot_2/cmd_vel', Twist, queue_size=100)
		self.image = Image()

	def	looker(self, msg):
		self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_green = numpy.array([45, 100, 50])
		upper_green = numpy.array([75, 255, 255])
		lower_yellow = numpy.array([15, 100, 50])
		upper_yellow = numpy.array([35, 255, 255])
		lower_red1 = numpy.array([0, 100, 50])
		upper_red1 = numpy.array([10, 255, 255])
		lower_red2 = numpy.array([160, 100, 50])
		upper_red1 = numpy.array([179, 255, 255])
		lower_blue = numpy.array([105, 100, 50])
		upper_blue = numpy.array([135, 255, 255])
		maskG = cv2.inRange(hsv, lower_green, upper_green)
		maskY = cv2.inRange(hsv, lower_yellow, upper_yellow)
		maskR = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
		maskB = cv2.inRange(hsv, lower_blue, upper_blue)
		colour = [maskG, maskY, maskR, maskB]
		
		for cm in colour:
			M = cv2.moments(cm)
			if M['m00'] > 0:
				self.mover(cm)
				break
		
		self.roaming()
		
	def roaming(self);
		self

	def mover(self, mask):
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
		
		cv2.imshow("window", image)
		cv2.imshow("mask", mask)
		cv2.waitKey(3)
		
rospy.init_node('hunter', anonymous=True)
Do = Searcher()
rospy.spin()