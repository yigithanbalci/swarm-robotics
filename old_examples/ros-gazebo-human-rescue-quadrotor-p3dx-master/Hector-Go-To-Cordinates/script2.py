#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy

from time import sleep
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt


class Robot():
	def __init__(self):
		self.pose = None
		self.odom = rospy.Subscriber("/ground_truth/state", Odometry, self.odom_callback)
		self.cmd  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.twist = Twist()
		self.rate = rospy.Rate(10)

		

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
				r = sqrt(p)
				#print(r)
				if(r < 0.1):				
					self.twist.linear.x = 0.0
					self.twist.linear.y = 0.0
					self.twist.linear.z = 0.0
				else:				
					self.twist.linear.x = 0.5 * (px -x) /r
					self.twist.linear.y = 0.5 * (py -y) /r
					self.twist.linear.z = 0.5 * (pz -z) /r

				self.twist.angular.x = 0.0
				self.twist.angular.y = 0.0 
				self.twist.angular.z = 0.0
			
				# Bir şey yap
				self.cmd.publish(self.twist)
				self.rate.sleep()
			else:
				pass

def main():
    #rospy.init_node("odometry_check")

    # subscriber = rospy.Subscriber("/ground_truth/state", Odometry, callback)
    # rospy.spin()
	
	rospy.init_node('Hector')
	#rospy.init_node('robot_controller')
	robot = Robot()

	robot.goto_point(10, 5, 20)


if __name__ == '__main__':
    main()
