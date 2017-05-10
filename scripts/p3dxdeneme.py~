#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import sys
import math
import tf as tf

from time import sleep
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians

class Robot():
    def __init__(self, name):
        self.pose = None
        self.orient = None
        self.euler = None
        self.name = name
        self.odom = rospy.Subscriber("/" + name + "/odom", Odometry, self.odom_callback)
        self.cmd  = rospy.Publisher("/" + name + '/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.rate = rospy.Rate(10)
    
    def odom_callback(self, data):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        self.pose = position
        self.pose.y = self.pose.y
        self.orient = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            )
        self.euler = tf.transformations.euler_from_quaternion(self.orient)
        print self.euler
        
    def goto_point(self, px, py, pz):
       # completed = False

        #twist = Twist()

       # while not completed:
            if self.pose != None:
                x = self.pose.x
                y = self.pose.y
                eu = self.euler[2]
                dist = ((px - x) ** 2 + (py - y) ** 2) ** 0.5
                angle = math.atan2(py - y, px - x)
    
                radius = dist * 0.5 / math.sin(eu - angle) if eu - angle != 0.0 else float('Inf')
    
                linear = angular = 0.0
                
                fx = px - x
                fy = py - y 
                
                field = return_field(fx, fy)
                
                if (field == 1):
                    yawangle = 
    
                if dist > 0.10:
                    linear = abs(radius) if abs(radius) < 1.0 else 1.0
                    angular = -linear / radius
                    angular *= 5.0
                    while(True):
                        if((eu-angle) < math.radians(10) or (eu-angle) > math.radians(-10)):
                            self.twist.angular.z = angular
                            self.cmd.publish(self.twist)
                            self.rate.sleep()
                        else:
                            self.twist.angular.z = 0.0
                            self.cmd.publish(self.twist)
                            self.rate.sleep()
                            break
                    
                    self.twist.linear.x = linear   
                    self.twist.angular.z = 0.0
                    self.cmd.publish(self.twist)
                else:
                    self.stop()
                
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
        
def return_field(x, y):
    if (x > 0 and y > 0):
        return 1
    elif (x < 0 and y > 0):
        return 2
    elif (x < 0 and y < 0):
        return 3
    elif (x > 0 and y < 0):
        return 4
    else: 
        return 5
    
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
