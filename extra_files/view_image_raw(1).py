# Bare Bones Code to View the Image Published from the Turtlebot3 on a Remote Computer
# Intro to Robotics Research 7785
# Georgia Institute of Technology
# Sean Wilson, 2022

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class MinimalVideoSubscriber(Node):

	def __init__(self):		
		# Creates the node.
		super().__init__('minimal_video_subscriber')

		# Set Parameters
		self.declare_parameter('show_image_bool', True)
		self.declare_parameter('window_name', "Raw Image")

		#Determine Window Showing Based on Input
		self._display_image = bool(self.get_parameter('show_image_bool').value)

		# Declare some variables
		self._titleOriginal = self.get_parameter('window_name').value # Image Window Title	
		
		#Only create image frames if we are not running headless (_display_image sets this)
		if(self._display_image):
		# Set Up Image Viewing
			cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE ) # Viewing Window
			cv2.moveWindow(self._titleOriginal, 50, 50) # Viewing Window Original Location
		
		#Set up QoS Profiles for passing images over WiFi
		image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

		#Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/image_raw/compressed',
				self._image_callback,
				image_qos_profile)
		self._video_subscriber # Prevents unused variable warning.

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
		self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
		
		cv2.namedWindow(window_capture_name)
		cv2.namedWindow(window_detection_name)

		# image processing from lab 1
		frame = self._imgBGR
		blur = cv.GaussianBlur(frame, (15, 15), 0)
		frame_HSV = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
		self.frame_threshold = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
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
			
		if(self._display_image):
			# Display the image in a window
			self.show_image(self._imgBGR)
			self.show_image(self.frame_threshold)
		
		

	def show_image(self, img):
		cv2.imshow(self._titleOriginal, img)
		# Cause a slight delay so image is displayed
		self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.

	def get_user_input(self):
		return self._user_input


def main():
	rclpy.init() #init routine needed for ROS2.
	video_subscriber = MinimalVideoSubscriber() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(video_subscriber) # Trigger callback processing.
		if(video_subscriber._display_image):	
			if video_subscriber.get_user_input() == ord('q'):
				cv2.destroyAllWindows()
				break

	#Clean up and shutdown.
	video_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()
