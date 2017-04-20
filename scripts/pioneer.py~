#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import sys
import math


from time import sleep
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians

class Robot():
    def __init__(self, name):
        self.pose = None
        self.name = name
        self.odom = rospy.Subscriber("/" + name + "/odom", Odometry, self.odom_callback)
        self.cmd  = rospy.Publisher("/" + name + '/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.rate = rospy.Rate(10)
    
    def odom_callback(self, data):
        position = data.pose.pose.position
        self.pose = position
        self.pose.y = self.pose.y

    def goto_point(self, px, py, pz):
       # completed = False

        #twist = Twist()

       # while not completed:
            if self.pose != None:
                x = self.pose.x
                y = self.pose.y
                z = self.pose.z
                #print x,y,z
                # Değerlendir
                p = (abs(px-x)**2) + (abs(py-y)**2)
                #print(p)
                r = math.sqrt(p)
                #print(r)
                xangle = math.acos((px-x)/r)
                yangle = math.acos((py-y)/r)
                #zangle = math.acos((pz-z)/r)
                #print(r)
                if(r < 0.1):                
                    self.twist.linear.x = 0.0
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                elif(r < 0.5):
                    self.twist.linear.x = math.cos(xangle) * 0.1
                    self.twist.linear.y = math.cos(yangle) * 0.1
                   # self.twist.linear.z = math.cos(zangle) * 0.1 
                    self.twist.linear.z = 0
                else:                
                    self.twist.linear.x = math.cos(xangle)
                    self.twist.linear.y = math.cos(yangle)
                   # self.twist.linear.z = math.cos(zangle)
                    self.twist.linear.z = 0

                if(xangle > math.radians(100)):
                    self.twist.linear.x = 0.0
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0 
                    self.twist.angular.z = 0.5
                elif(xangle < math.radians(80)):
                    self.twist.linear.x = 0.0
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0 
                    self.twist.angular.z = -0.5
                else:
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0 
                    self.twist.angular.z = 0.0
                # Bir şey yap
                self.cmd.publish(self.twist)
                self.rate.sleep()
            else:
                pass

    def control_help(self):
            while(True):
                try:
                    fo = open("/home/yigithan/catkin_ws/src/quadro_demo/communication/comm.txt", "r")
                    filestring = fo.readline()
                    if (filestring != None):
                        px, py, pz = filestring.split()
                        self.goto_point(float(px), float(py), float(pz))
                        #print float(px), float(py), float(pz)
                    fo.close()
                except:
                    pass
        
def main(name):
    #rospy.init_node("odometry_check")

    # subscriber = rospy.Subscriber("/ground_truth/state", Odometry, callback)
    # rospy.spin()
    
    rospy.init_node('P3dx', name)
    #rospy.init_node('robot_controller')
    robot = Robot(name)
    robot.control_help()

if __name__ == '__main__':
    main(sys.argv[1])
