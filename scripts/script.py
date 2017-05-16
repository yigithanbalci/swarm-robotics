#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import tf
import thread

from time import sleep
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage

import math
import sys
import os, time

import numpy as np
import cv2

class Robot():
    def __init__(self, name):
        self.pose = None
        self.orient = None
        self.euler = None
        self.insan = 0
        self.bekle = 0
        self.takip = False
        self.ilkgorus = True
        self.name = name
        self.odom = rospy.Subscriber("/" + name + "/ground_truth/state", Odometry, self.odom_callback)
        self.cmd  = rospy.Publisher("/" + name + '/cmd_vel', Twist, queue_size=10)
        self.cam = rospy.Subscriber("/" + name + "/front_cam/camera/image/compressed", CompressedImage, self.cam_callback)
        self.twist = Twist()
        self.filepath = "/home/yigit/catkin_ws/src/quadro_demo/src/swarm-robotics/camera" + "/" + self.name
        self.timestr = time.strftime("%Y%m%d-%H%M%S")
        self.rate = rospy.Rate(10)

        

    def cam_callback(self, cam_data):
        #print("received data type: " + cam_data.format)
        self.timestr = time.strftime("%Y%m%d-%H%M%S")
                
        #converts image to cv2
        np_arr = np.fromstring(cam_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        image_np = hog_human_detection(self, image_np)
        
        # w 320 , h 240
        
        #saves image
        if cam_data.format == "rgb8; jpeg compressed bgr8":
            if self.insan == 1:
                cv2.imwrite(self.filepath + "/" + self.name + "-" + self.timestr + ".jpeg", image_np)
                self.insan = 0
        #shows image
        cv2.imshow(self.name, image_np)
        cv2.waitKey(2)
        
        #creates path if not exist but it dont do this from .launch
        if not os.path.exists(self.filepath):
            os.makedirs(self.filepath)
        
    def odom_callback(self, data):
        position = data.pose.pose.position
        self.pose = position
        orientation = data.pose.pose.orientation
        self.orient = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            )
        self.euler = tf.transformations.euler_from_quaternion(self.orient)
        #print self.euler

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
                #print(r)
                
                if px-x == 0:
                    yonx = 0
                elif px-x < 0:
                    yonx = -1
                else:
                    yonx = 1
                
                if py-y == 0:
                    yony = 0
                elif py-y < 0:
                    yony = -1
                else:
                    yony = 1
                
                if pz-z == 0:
                    yonz = 0
                elif pz-z < 0:
                    yonz = -1
                else:
                    yonz = 1
                
                '''
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
                    self.twist.linear.z = 0.1 
                else:                
                    self.twist.linear.x = math.cos(xangle)
                    self.twist.linear.y = math.cos(yangle)
                    self.twist.linear.z = 1.0
                    '''
                if self.bekle == 0:
                    if(px-x < 0.1):
                        self.twist.linear.x = 0.0 * yonx
                    elif(px-x < 0.5):
                        self.twist.linear.x = 0.1 * yonx
                    elif(px-x < 1):
                        self.twist.linear.x = 0.5 * yonx
                    elif(px-x > 1):
                        self.twist.linear.x = 1.0 * yonx
                        
                    if(py-y < 0.1):
                        self.twist.linear.y = 0.0 * yony
                    elif(py-y < 0.5):
                        self.twist.linear.y = 0.1 * yony
                    elif(py-y > 0.5):
                        self.twist.linear.y = 1.0 * yony
                    elif(py-y > 1):
                        self.twist.linear.y = 1.0 * yony
                        
                    if(pz-z < 0.1):
                        self.twist.linear.z = 0.0 * yonz
                    elif(pz-z < 0.5):
                        self.twist.linear.z = 0.1 * yonz
                    elif(pz-z > 0.5):
                        self.twist.linear.z = 1.0 * yonz
                    elif(pz-z > 1):
                        self.twist.linear.z = 1.0 * yonz
                elif self.bekle == 1:
                    self.twist.linear.x = 0.1 * yonx
                    self.twist.linear.y = 0.1 * yony
                    self.twist.linear.z = 0.1 * yonz
                    if(px-x < 0.1):
                        self.twist.linear.x = 0.0 * yonx
                    if(py-y < 0.1):
                        self.twist.linear.y = 0.0 * yony
                    if(pz-z < 0.1):
                        self.twist.linear.z = 0.0 * yonz
                    self.cmd.publish(self.twist)
                    self.rate.sleep()
                    #time.sleep(2)
                    while self.takip:
                        pass
                    
                self.twist.angular.x = 0.0
                self.twist.angular.y = 0.0 
                self.twist.angular.z = 0.0
            
            
                # Bir şey yap
                self.cmd.publish(self.twist)
                self.rate.sleep()
            else:
                pass


def inside(r, q):
    rx, ry, rw, rh = r
    qx, qy, qw, qh = q
    return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh


def draw_detections(img, rects, thickness = 1):
    for x, y, w, h in rects:
        # the HOG detector returns slightly larger rectangles than the real objects.
        # so we slightly shrink the rectangles to get a nicer output.
        pad_w, pad_h = int(0.15*w), int(0.05*h)
        cv2.rectangle(img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), (0, 255, 0), thickness)
        

def hog_human_detection(self, img):

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector( cv2.HOGDescriptor_getDefaultPeopleDetector() )
    found = None
    found,w=hog.detectMultiScale(img, winStride=(8,8), padding=(32,32), scale=1.05)
    foundCounter = 0

    for x, y, w, h in found:
        foundCounter = foundCounter + 1
            
    if(foundCounter != 0):
        send_help(self)
        self.insan = 1
        self.bekle = 1
        self.takip = True
        pix = (160-(x+(w/2)))
        pix2 = (120-(y+(h/2)))
        print str(pix2) + time.strftime("%c")   
        if self.ilkgorus == True:
            #time.sleep(2)
            self.ilkgorus = False
        thread.start_new_thread(track_human_center, ("TrackThread", self,(x+(w/2)), (y+(h/2))))
    elif(foundCounter == 0):
        self.takip = False
        self.bekle = 0
        self.insan = 0
        self.ilkgorus = True
    draw_detections(img,found)
    #cv2.imshow('feed',img)
    #cv2.destroyAllWindows()
    return img

def send_help(self):
    if self.pose != None:
        x = self.pose.x
        y = self.pose.y
        z = self.pose.z
    try:
        fo = open("/home/yigit/catkin_ws/src/quadro_demo/src/swarm-robotics/communication/comm.txt", "a")
        filestring = "\n%.10f %.10f %.10f" % ( (x), (y), (z))
        fo.write(filestring)
        fo.close()
    except:
        pass
    
    
def track_human_center(threadName, self, humanx, humany):
    if(120-humany) == 0:
        yonx = 0
    elif (120-humany) < 0:
        yonx = -1
    else:
        yonx = 1
    
    if(160-humanx) == 0:
        yony = 0
    elif (160-humanx) < 0:
        yony = -1
    else:
        yony = 1
    
    farkx = 160-humanx
    farky = 120-humany
    
    if farkx < 0:
        farkx = farkx * (-1)
        
    if farkx > 100:
        hizx = 1.0
    elif farkx > 90:
        hizx = 0.9
    elif farkx > 80:
        hizx = 0.8
    elif farkx > 70:
        hizx = 0.7
    elif farkx > 60:
        hizx = 0.6
    elif farkx > 50:
        hizx = 0.5
    elif farkx > 40:
        hizx = 0.4
    elif farkx > 30:
        hizx = 0.3
    elif farkx > 20:
        hizx = 0.2
    elif farkx > 8:
        hizx = 0.1
    else:
        hizx = 0.0
    
    if farky < 0:
        farky = farky * (-1)
    '''   
    if farky > 100:
        hizy = 1.0
    elif farky > 90:
        hizy = 0.9
    elif farky > 80:
        hizy = 0.8
    elif farky > 70:
        hizy = 0.7
    elif farky > 60:
        hizy = 0.6
    elif farky > 50:
        hizy = 0.5
    elif farky > 40:
        hizy = 0.4
    elif farky > 30:
        hizy = 0.3
    elif farky > 20:
        hizy = 0.2
    elif farky > 8:
        hizy = 0.1
    else:
        hizy = 0.0
    '''
        
    if farky > 30 and farky < 40:
        hizy = 0.0
    elif farky < 26:
        hizy = 0.2
    elif farky > 46:
        hizy = 0.2
    else:
        hizy = 0.0
    
    self.twist.linear.x = yonx * hizy
    self.twist.linear.y = yony * hizx
    self.cmd.publish(self.twist)
    self.rate.sleep()
    
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