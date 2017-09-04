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
from sensor_msgs.msg import LaserScan

import math
import sys
import os, time

import numpy as np
import cv2
from __builtin__ import float
from cmath import cos

class Robot():
    def __init__(self, name):
        self.pose = None
        self.oldtime = None
        self.orient = None
        self.euler = None
        self.ilkgoruspose = None
        self.takipbirakmaani = None
        self.goback = False
        self.takipbirakmapose = None
        self.insanpose = None
        self.insan = 0
        self.bekle = 0
        self.takip = False
        self.takip2 = False
        self.ilkgorus = True
        self.name = name
        self.eskipose = None
        self.oldlaser = Laser()
        self.newlaser = Laser()
        self.lasertime = time.time()
        self.kacmazamani = None
        self.hedefyerdegisimi = None
        self.target = Target()
        self.insanalgilandi = False
        self.odom = rospy.Subscriber("/" + name + "/ground_truth/state", Odometry, self.odom_callback, queue_size=10)
        self.cmd  = rospy.Publisher("/" + name + '/cmd_vel', Twist, queue_size=10)
        self.cam = rospy.Subscriber("/" + name + "/front_cam/camera/image/compressed", CompressedImage, self.cam_callback, queue_size=10)
        self.lsr = rospy.Subscriber("/" + name + "/scan", LaserScan, self.laser_callback, queue_size=10)
        self.twist = Twist()
        self.laser = LaserScan()
        self.filepath = "/home/yigit/catkin_ws/src/quadro_demo/src/swarm-robotics/camera" + "/" + self.name
        self.timestr = time.strftime("%Y%m%d-%H%M%S")
        self.rate = rospy.Rate(10)
        
    def laser_callback(self, laser_data):
        self.laser = laser_data
        min = 35 
        angle_min = self.laser.angle_min
        angle_max = self.laser.angle_max
        angle_increment = self.laser.angle_increment
        #laser angles could be set by calculation of human's angle using detected human from image.
        #it can be more precise.
        search_angle_min = -45 * (math.pi/180)
        search_angle_max = 45 * (math.pi/180)
        
        s_angle_min = search_angle_min - angle_min
        s_angle_max = search_angle_max - angle_min
        
        index_min = s_angle_min / angle_increment
        index_max = s_angle_max / angle_increment 
        
        index_min_int = int(math.ceil(index_min))
        index_max_int = int(math.ceil(index_max))
    
        
        if time.time() - self.lasertime >= 1:    
            if self.newlaser.check_notnone() :
                if self.newlaser.check_notnone() and self.oldlaser.check_notnone(): 
                    yerdegistirme = (abs(self.newlaser.x - self.oldlaser.x)**2) + (abs(self.newlaser.y - self.oldlaser.y)**2)
                    yerdegistirme = math.sqrt(yerdegistirme)
                    yerdegistirme = float(format(yerdegistirme, '.3f'))
                    self.hedefyerdegisimi = yerdegistirme
                
                self.oldlaser.equalize_objects(self.newlaser)
            self.lasertime = time.time()
        
        i = index_min_int
        while i<=index_max_int:
            if self.laser.ranges[i] > 1:
                    if self.laser.ranges[i] < min:
                        min  = self.laser.ranges[i]
            i = i+1
            
        i = index_min_int
        minindex = index_max_int
        maxindex = index_min_int
        
        while i <= index_max_int:
            if self.laser.ranges[i] > 1:
                if self.laser.ranges[i] <= min :
                    if minindex > i:
                        minindex = i
                        maxindex = i + 3
                        break
                    if maxindex < i:
                        maxindex = i
                        minindex = i - 3                        
            i = i+1
        self.newlaser.mindist = min
        self.newlaser.maxindex = maxindex
        self.newlaser.minindex = minindex
        self.newlaser.angle = ((((maxindex + minindex) / 2)*angle_increment) + angle_min)
        self.newlaser.angle = self.newlaser.angle * (-1)
        self.newlaser.betwangle = self.newlaser.angle
        angle = float(format(self.newlaser.angle, '.3f')) + float(format(self.euler[2], '.3f'))
        self.newlaser.angle = angle
        x = math.cos(angle) * float(format(self.newlaser.mindist, '.3f'))
        y = math.sin(angle) * float(format(self.newlaser.mindist, '.3f'))
        rx = float(format(x, '.3f')) + float(format(self.pose.x, '.3f'))
        ry = float(format(y, '.3f')) + float(format(self.pose.y, '.3f'))
        self.newlaser.x = rx
        self.newlaser.y = ry
        
    def cam_callback(self, cam_data):
        self.timestr = time.strftime("%Y%m%d-%H%M%S")
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

    def goto_point(self, px, py, pz):
        completed = False
        self.target.x = px
        self.target.y = py
        self.target.z = pz

        while not completed:
            if self.pose != None:
                x = self.pose.x
                y = self.pose.y
                z = self.pose.z
                eu = 0.0
                if self.euler != None:
                    eu = self.euler[2]
                    eus = format(eu, '.2f')
                    eu = float(eus)

                p = (abs(px-x)**2) + (abs(py-y)**2) + (abs(pz-z)**2)
                r = math.sqrt(p)
                if r < 0.5:
                    completed = True
                                    
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
                
                if self.bekle == 0:
                    farkx = abs(px-x)
                    farky = abs(py-y)
                    farkz = abs(pz-z)
                    
                    if(farkx < 0.1):
                        self.twist.linear.x = 0.0 * yonx
                    elif(farkx < 0.5):
                        self.twist.linear.x = 0.1 * yonx
                    elif(farkx < 1):
                        self.twist.linear.x = 0.5 * yonx
                    elif(farkx > 1):
                        self.twist.linear.x = 1.0 * yonx
                        
                    if(farky < 0.1):
                        self.twist.linear.y = 0.0 * yony
                    elif(farky < 0.5):
                        self.twist.linear.y = 0.1 * yony
                    elif(farky > 0.5):
                        self.twist.linear.y = 1.0 * yony
                    elif(farky > 1):
                        self.twist.linear.y = 1.0 * yony
                        
                    if(farkz < 0.1):
                        self.twist.linear.z = 0.0 * yonz
                    elif(farkz < 0.5):
                        self.twist.linear.z = 0.1 * yonz
                    elif(farkz > 0.5):
                        self.twist.linear.z = 1.0 * yonz
                    elif(farkz > 1):
                        self.twist.linear.z = 1.0 * yonz
                elif self.bekle == 1:
                    farkx = abs(px-x)
                    farky = abs(py-y)
                    farkz = abs(pz-z)
                    
                    self.twist.linear.x = 0.1 * yonx
                    self.twist.linear.y = 0.1 * yony
                    self.twist.linear.z = 0.1 * yonz
                    if(farkx < 0.1):
                        self.twist.linear.x = 0.0 * yonx
                    if(farky < 0.1):
                        self.twist.linear.y = 0.0 * yony
                    if(farkz < 0.1):
                        self.twist.linear.z = 0.0 * yonz
                    self.cmd.publish(self.twist)
                    self.rate.sleep()
                    while self.takip:
                        pass
                    
                self.twist.angular.x = 0.0
                self.twist.angular.y = 0.0 

                self.twist.angular.z = (1.7 * eu) / (math.pi * (-1))
                
                self.twist.linear.x = (self.twist.linear.x * math.cos(eu)) + (self.twist.linear.y * math.sin(eu))
                self.twist.linear.y = (self.twist.linear.x * math.sin(eu)) + (self.twist.linear.y * math.cos(eu))
                if self.newlaser.mindist <=2:
                    self.twist.linear.z = 1.0

                self.cmd.publish(self.twist)
                self.rate.sleep()
            else:
                pass
        return True

class Laser():
    def __init__(self):
        self.mindist = None
        self.minindex = None
        self.maxindex = None
        self.angle = None
        self.betwangle = None
        self.x = None
        self.y = None
    def check_notnone(self):
        if self.mindist == None and self.angle == None and self.maxindex == None and self.minindex == None and self.x == None and self.y == None and self.betwangle == None:
            return False
        else:
            return True
    def equalize_objects(self, stranger):
        self.mindist = stranger.mindist
        self.minindex = stranger.minindex
        self.maxindex = stranger.maxindex
        self.angle = stranger.angle
        self.x = stranger.x
        self.y = stranger.y
        
class Target():
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        
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
        self.insanalgilandi = True
        if self.takip2 == False:
            self.takip2 = True
            self.takip = True
            thread.start_new_thread(insan_takibi, ("TakipThread", self))
    
    elif foundCounter == 0:
        self.insanalgilandi = False
        self.takip2 = False
        self.takip = False

    draw_detections(img,found)
    return img

def insan_takibi(threadName, self):
    ex = self.pose.x
    ey = self.pose.y       
    if self.hedefyerdegisimi > 0.1:
        completed = False
        
        while not completed:
            eu = self.euler[2]
            eus = format(eu, '.2f')
            eu = float(eus)

            x = self.newlaser.x
            y = self.newlaser.y
            
            x = x - (math.cos(self.newlaser.angle) * 1.0)
            y = y - (math.sin(self.newlaser.angle) * 1.0)
            
            comp = goto_point(self,True, x, y, 1.8, self.newlaser.betwangle)

            if comp == True and self.hedefyerdegisimi < 0.1:
                completed = True
                self.takip = False
                self.kacmazamani = time.time()
                self.takip2 = False
                return
            
    if self.hedefyerdegisimi < 0.1:        
        eu = self.euler[2]
        eus = format(eu, '.2f')
        eu = float(eus)
        
        x = self.newlaser.x + (math.cos(eu) * self.newlaser.mindist)
        y = self.newlaser.y + (math.sin(eu) * self.newlaser.mindist)
        cmp = goto_point(self,False, x, y, 3.0, 0.0)

    self.kacmazamani = time.time()
    self.takip = False
    self.takip2 = False
    
def noktalar_arasi_yon(a, b):
    if a < b:
        return 1
    else:
        return -1
def goto_point(self,insantakip, px, py, pz, aci):
    completed = False

    while not completed:
        if self.pose != None:
            x = self.pose.x
            y = self.pose.y
            z = self.pose.z

            if insantakip:
                eu = self.euler[2]
                eus = format(eu, '.2f')
                eu = float(eus)

                px = self.newlaser.x
                py = self.newlaser.y
                px = px - (math.cos(self.newlaser.angle) * 1.0)
                py = py - (math.sin(self.newlaser.angle) * 1.0)
                aci = self.newlaser.betwangle
            else:
               pass
           
            p = (abs(px-x)**2) + (abs(py-y)**2) + (abs(pz-z)**2)
            r = math.sqrt(p)
            if r <= 1 or (self.hedefyerdegisimi < 0.1 and self.insanalgilandi == False):
                completed = True
                self.twist.linear.x = 0.0
                self.twist.linear.y = 0.0
                self.twist.linear.z = 0.0
                self.twist.angular.z = 0.0
        
                self.cmd.publish(self.twist)
                self.rate.sleep()
                return True
            
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
            
            if self.bekle == 0:
                farkx = abs(px-x)
                farky = abs(py-y)
                farkz = abs(pz-z)
                
                if(farkx <= 0.1):
                    self.twist.linear.x = 0.0 * yonx
                elif(farkx <= 0.5):
                    self.twist.linear.x = 0.1 * yonx
                elif(farkx < 1):
                    self.twist.linear.x = 0.5 * yonx
                elif(farkx >= 1):
                    self.twist.linear.x = 1.0 * yonx
                    
                if(farky <= 0.1):
                    self.twist.linear.y = 0.0 * yony
                elif(farky <= 0.5):
                    self.twist.linear.y = 0.1 * yony
                elif(farky < 1):
                    self.twist.linear.y = 0.5 * yony
                elif(farky >= 1):
                    self.twist.linear.y = 1.0 * yony
                    
                if(farkz <= 0.1):
                    self.twist.linear.z = 0.0 * yonz
                elif(farkz <= 0.5):
                    self.twist.linear.z = 0.1 * yonz
                elif(farkz < 0.5):
                    self.twist.linear.z = 0.5 * yonz
                elif(farkz >= 1):
                    self.twist.linear.z = 1.0 * yonz
            elif self.bekle == 1:
                farkx = abs(px-x)
                farky = abs(py-y)
                farkz = abs(pz-z)
                
                self.twist.linear.x = 0.1 * yonx
                self.twist.linear.y = 0.1 * yony
                self.twist.linear.z = 0.1 * yonz
                if(farkx < 0.1):
                    self.twist.linear.x = 0.0 * yonx
                if(farky < 0.1):
                    self.twist.linear.y = 0.0 * yony
                if(farkz < 0.1):
                    self.twist.linear.z = 0.0 * yonz
                self.rate.sleep()
                
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0 
            self.twist.angular.z = 0.0
            
            if(farkx <= 0.1):
                self.twist.linear.x = 0.0 * yonz
            else:
                self.twist.linear.x = yonx * clamp(abs(farkx)/2, 0.1, 1.0)
                
            if(farky <= 0.1):
                self.twist.linear.y = 0.0 * yonz
            else:
                self.twist.linear.y = yony * clamp(abs(farky)/2, 0.1, 1.0)
                
            if(farkz <= 0.1):
                self.twist.linear.z = 0.0 * yonz        
            else:
                self.twist.linear.z = yonz * clamp(abs(farkz)/2, 0.1, 1.0)
            
            if insantakip:
                carpan = 0
                if aci < 0:
                    carpan = -1
                elif aci > 0:
                    carpan = 1
                else:
                    carpan = 0
                    
                self.twist.angular.z = (1.7 * self.newlaser.betwangle) / (math.radians(135))

            if completed:
                self.twist.linear.x = 0.0
                self.twist.linear.y = 0.0
                self.twist.linear.z = 0.0
                self.twist.angular.z = 0.0
        
            self.twist.linear.x = (self.twist.linear.x * math.cos(eu)) + (self.twist.linear.y * math.sin(eu))
            self.twist.linear.y = (self.twist.linear.x * math.sin(eu)) + (self.twist.linear.y * math.cos(eu))
                
            self.cmd.publish(self.twist)
            self.rate.sleep()
        else:
            pass
    return True

def clamp(n, minn, maxn):
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n
    
def insandan_uzaklasma(eskipose, pose, mesafe):
    
    if pose != None:
        x = pose.x
        y = pose.y
        z = pose.z
        
    if eskipose != None:
        ex = eskipose.x
        ey = eskipose.y
        ez = eskipose.z
        
    p = (abs(x-ex)**2) + (abs(ey-y)**2) + (abs(ez-z)**2)
    
    r = math.sqrt(p)
    
    if r > mesafe:
        return True
    else:
        return False

def yer_degistirme(eskipose, pose):
    
    if pose != None:
        x = pose.x
        y = pose.y
        z = pose.z
        
    if eskipose != None:
        ex = eskipose.x
        ey = eskipose.y
        ez = eskipose.z
        
    p = (abs(x-ex)**2) + (abs(ey-y)**2) + (abs(ez-z)**2)
    
    r = math.sqrt(p)
    
    if r > 0.5:
        return True
    else:
        return False
    
def return_field(eu):
    if eu > 0.0 and eu < 1.57:
        return 1
    elif eu > 1.57 and eu < 3.14:
        return 2
    elif eu > -3.14 and eu < -1.57:
        return 3
    elif eu > -1.57 and eu < 0.0:
        return 4
    elif eu == 0.0:
        return 5
    elif eu == 1.57:
        return 6
    elif eu == -1.57:
        return 8
    elif eu == 3.14:
        return 7
    elif eu == -3.14:
        return 7
    
def send_help(threadName, self, dist):
    if self.pose != None:
        x = self.pose.x
        y = self.pose.y
        z = self.pose.z
    eu = self.euler[2]
    eus = format(eu, '.2f')
    eu = float(eus)
    self.insanpose = self.pose
    
    self.insanpose.x = self.pose.x + (math.cos(eu) * dist)
    self.insanpose.y = self.pose.y + (math.sin(eu) * dist)
    self.insanpose.z = 0
    
    try:
        if control_help(self) is True:
            fo = open("/home/yigit/catkin_ws/src/quadro_demo/src/swarm-robotics/communication/comm.txt", "a")
            filestring = "\n%.10f %.10f %.10f" % ( (x), (y), (z))
            fo.write(filestring)
            fo.close()
        else:
            pass
    except:
        pass
    
def control_help(self):
    if self.insanpose != None:
        ax = self.insanpose.x
        ay = self.insanpose.y
        az = self.insanpose.z
    rvalue = True
        
    with open('/home/yigit/catkin_ws/src/quadro_demo/src/swarm-robotics/communication/comm.txt') as openfileobject:
        for line in openfileobject:
            if line != None and line != '\n':
                px, py, pz = line.split()
                x = float(px)
                y = float(py)
                z = float(pz)
                
                p = (abs(ax-x)**2) + (abs(ay-y)**2) + (abs(az-z)**2)
                r = math.sqrt(p)
                if r < 1:
                    rvalue = False
    return rvalue
    
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
    
    farkx = abs(160-humanx)
    farky = abs(120-humany)
    
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
    elif farkx > 15:
        hizx = 0.1
    elif farkx <= 15:
        hizx = 0.0
    
    if farky < 0:
        farky = farky * (-1)
        
    if farky > 29 and farky < 41:
        hizy = 0.0
    elif farky <= 20 and farky >= 10:
        hizy = 0.1
    elif farky >= 50 and farky <= 60:
        hizy = 0.1
    elif farky < 10:
        hizy = 0.2
    elif farky > 50:
        hizy = 0.2
    else:
        hizy = 0.0
        
    self.twist.linear.x = yonx * hizy
    self.twist.linear.y = yony * hizx
    self.cmd.publish(self.twist)
    self.rate.sleep()
    
def distBul (wi, hi):
    frame_size = wi * hi
    if frame_size >= 11523:
        dist = 2   #veya daha kucuk

    elif frame_size < 11523 and frame_size > 7700:
        #dist = 2 ile 2.5 arası        
        if frame_size > 9600:
            dist = 2.5
        else:
            dist = 2.25
        

    elif frame_size < 7700 and frame_size > 6528:
         #dist 2.5 ile 3 arasında
        if frame_size > 7100:
            dist = 2.75
        else:
            dist = 3.0

    elif frame_size < 6528 and frame_size > 5593:
        #dist 3 ile 3.5 arasında
        if frame_size > 6000:
            dist = 3.25
        else:
            dist = 3.5

    elif frame_size < 5593:
        #dist 3.5ten buyuk
        dist = 4.0
    
    return dist

def gobackto_point(threadName, self, px, py, pz, dist):
    completed = False
    self.goback = True
    thread.start_new_thread(send_help, ("sendHelpThread", self, dist))

    while not completed:
        if self.pose != None:
            x = self.pose.x
            y = self.pose.y
            z = self.pose.z

            p = (abs(px-x)**2) + (abs(py-y)**2) + (abs(pz-z)**2)
            r = math.sqrt(p)
            if r < 0.5:
                completed = True
                self.goback = False
                self.takip = False
                return True
            
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
            
            farkx = abs(px-x)
            farky = abs(py-y)
            farkz = abs(pz-z)
            
            if self.bekle == 0:
                if(farkx < 0.1):
                    self.twist.linear.x = 0.0 * yonx
                elif(farkx < 0.5):
                    self.twist.linear.x = 0.1 * yonx
                elif(farkx < 1):
                    self.twist.linear.x = 0.5 * yonx
                elif(farkx > 1):
                    self.twist.linear.x = 1.0 * yonx
                    
                if(farky < 0.1):
                    self.twist.linear.y = 0.0 * yony
                elif(farky < 0.5):
                    self.twist.linear.y = 0.1 * yony
                elif(farky > 0.5):
                    self.twist.linear.y = 1.0 * yony
                elif(farky > 1):
                    self.twist.linear.y = 1.0 * yony
                    
                if(farkz < 0.1):
                    self.twist.linear.z = 0.0 * yonz
                elif(farkz < 0.5):
                    self.twist.linear.z = 0.1 * yonz
                elif(farkz > 0.5):
                    self.twist.linear.z = 1.0 * yonz
                elif(farkz > 1):
                    self.twist.linear.z = 1.0 * yonz
            elif self.bekle == 1:
                self.twist.linear.x = 0.1 * yonx
                self.twist.linear.y = 0.1 * yony
                self.twist.linear.z = 0.1 * yonz
                if(farkx < 0.1):
                    self.twist.linear.x = 0.0 * yonx
                if(farky < 0.1):
                    self.twist.linear.y = 0.0 * yony
                if(farkz < 0.1):
                    self.twist.linear.z = 0.0 * yonz
                self.cmd.publish(self.twist)
                self.rate.sleep()                
                
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0 
            self.twist.angular.z = 0.0
                
            self.cmd.publish(self.twist)
            self.rate.sleep()
        else:
            pass
    self.goback = False
    self.takip = False
    return True
    
def main(name, sx, sy, sz):
    rospy.init_node('Hector', name)
    robot = Robot(name)

    hedef = robot.goto_point(float(sx), float(sy), float(sz))
    if hedef == True:
        pass

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])