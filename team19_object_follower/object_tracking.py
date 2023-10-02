# Joseph Sommer and Yash Mhaskar
from __future__ import print_function
import cv2 as cv
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import sys
import numpy as np
from cv_bridge import CvBridge

class ObjectTrackingPubsub(Node):
	def __init__(self):
		super().__init__('object_tracking_pub_sub')
		#Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
		self._get_image = self.create_subscription(
				CompressedImage,
				'/image_raw/compressed',
				self._image_callback,
				image_qos_profile)
		self._get_image # Prevents unused variable warning.

		self.dir_publisher_ = self.create_publisher(Int32, 'direction', 5)
		self.control_input = 0  # Default direction
		#self.timer = self.create_timer(0.5, self.publish_command)

		# image compression subscriber from Raw_image
		#Set up QoS Profiles for passing images over WiFi
		#image_qos_profile = QoSProfile(
		#    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		#    history=QoSHistoryPolicy.KEEP_LAST,
		#    durability=QoSDurabilityPolicy.VOLATILE,
		#    depth=1
		#)

	#def update_direction(self, direction):
	#     self.control_input = direction
	#     msg = Int32()
	#     msg.data = self.control_input
	#     self.publisher_.publish(msg)

	 def _image_callback(self, CompressedImage):
		# Variables
		max_value = 255
		max_value_H = 360//2
		low_H = 0
		low_S = 0
		low_V = 0
		high_H = max_value_H
		high_S = max_value
		high_V = max_value
		window_capture_name = 'Video Capture'
		window_detection_name = 'Object Detection'
		low_H_name = 'Low H'
		low_S_name = 'Low S'
		low_V_name = 'Low V'
		high_H_name = 'High H'
		high_S_name = 'High S'
		high_V_name = 'High V'
		radius = 0
		x_axis = 0
		y_axis = 0
		x_axis_max = 638
		y_axis_max = 478

		turn_dir = 0 # -1 is left, +1 is right
		#cap = cv.VideoCapture(args.camera)
		# Blue ball specific values
		low_H = 85
		low_S = 153
		low_V = 76
		high_H = 146

		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
		frame = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")

		# image processing from lab 1
		blur = cv.GaussianBlur(frame, (15, 15), 0)
		frame_HSV = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
		frame_threshold = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
		# finding the contours
		contours, _ = cv.findContours(frame_threshold, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
		# take the first contour
		count = 0
		# Prevents error if nothing is detected
		if len(contours)!=0:
			count = contours[0]
			(x_axis,y_axis),radius = cv.minEnclosingCircle(count)
			center = (int(x_axis),int(y_axis))
			radius = int(radius)
		# reduces likelihood of showing contour on wrong object
		if radius>40:
			cv.circle(frame,center,radius,(0,255,0),2)
			cv.circle(frame_threshold,center,radius,(0,255,0),2)

		#print center point of object
		#print("Center: %2d, %2d" % (x_axis, y_axis))
		if x_axis>(x_axis_max/2 -50) and x_axis<(x_axis_max/2 +50):
			turn_dir = 0
		elif x_axis<(x_axis_max/2):
			turn_dir = -1
		elif x_axis>(x_axis_max/2):
			turn_dir = 1
		#print("Direction: %2d" % (turn_dir))\
		#self.publish_.publish(turn_dir)
		#coord_publisher.update_direction(turn_dir)
		# publish direction
		self.dir_publisher_.publish(turn_dir)
		wait(100)


def main(args=None):
	# Setting up publisher values
	rclpy.init(args=args)
	object_tracking_pub_sub=ObjectTrackingPubsub()
	rclpy.spin(object_tracking_pub_sub)

	coord_publisher.destroy_node()
	rclpy.shutdown()

if __name__=='__main__':
  main()
