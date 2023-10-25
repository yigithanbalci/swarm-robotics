#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy

from time import sleep
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage

import math
import sys
import os, time

import numpy as np
import cv2
from __builtin__ import file
from twisted.python.filepath import FilePath

class Robot():
	def __init__(self, name):
		self.pose = None
		self.name = name
		self.odom = rospy.Subscriber("/" + name + "/ground_truth/state", Odometry, self.odom_callback)
		self.cmd  = rospy.Publisher("/" + name + '/cmd_vel', Twist, queue_size=10)
		self.cam = rospy.Subscriber("/" + name + "/front_cam/camera/image/compressed", CompressedImage, self.cam_callback)
		self.twist = Twist()
		self.filepath = "/home/yigithan/catkin_ws/src/quadro_demo/camera" + "/" + self.name
		self.timestr = time.strftime("%Y%m%d-%H%M%S")
		self.rate = rospy.Rate(10)

		

	def cam_callback(self, cam_data):
		print("received data type: " + cam_data.format)
		self.timestr = time.strftime("%Y%m%d-%H%M%S")
				
		#converts image to cv2
		np_arr = np.fromstring(cam_data.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
		
		#saves image
		if cam_data.format == "rgb8; jpeg compressed bgr8":
			cv2.imwrite(self.filepath + "/" + self.name + "-" + self.timestr + ".jpeg", image_np)
		
		#shows image
		cv2.imshow('cv_img', image_np)
		cv2.waitKey(2)
		
		#creates path if not exist but it dont do this from .launch
		if not os.path.exists(self.filepath):
			os.makedirs(self.filepath)
		
	def odom_callback(self, data):
		position = data.pose.pose.position
		self.pose = position

	def goto_point(self, px, py, pz):
		completed = False

		#twist = Twist()

		while not completed:
			if self.pose != None:
				x = self.pose.x
				y = self.pose.y
				z = self.pose.z

				# Değerlendir
				p = (abs(px-x)**2) + (abs(py-y)**2) + (abs(pz-z)**2)
				#print(p)
				r = math.sqrt(p)
				print(r)
				xangle = math.acos((px-x)/r)
				yangle = math.acos((py-y)/r)
				zangle = math.acos((pz-z)/r)
				#print(r)
				if(r < 0.1):				
					self.twist.linear.x = 0.0
					self.twist.linear.y = 0.0
					self.twist.linear.z = 0.0
				elif(r < 0.5):
					self.twist.linear.x = math.cos(xangle) * 0.1
					self.twist.linear.y = math.cos(yangle) * 0.1
					self.twist.linear.z = math.cos(zangle) * 0.1 
				else:				
					self.twist.linear.x = math.cos(xangle)
					self.twist.linear.y = math.cos(yangle)
					self.twist.linear.z = math.cos(zangle)

				self.twist.angular.x = 0.0
				self.twist.angular.y = 0.0 
				self.twist.angular.z = 0.0
			
				# Bir şey yap
				self.cmd.publish(self.twist)
				self.rate.sleep()
			else:
				pass

def main(name, sx, sy, sz):
    #rospy.init_node("odometry_check")

    # subscriber = rospy.Subscriber("/ground_truth/state", Odometry, callback)
    # rospy.spin()
	
	rospy.init_node('Hector', name)
	#rospy.init_node('robot_controller')
	robot = Robot(name)

	robot.goto_point(float(sx), float(sy), float(sz))


if __name__ == '__main__':
	main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
