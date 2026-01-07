#!/usr/bin/env python3
# encoding: utf-8
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CompressedImage,Image
from std_msgs.msg import Float32, Bool,UInt16,Int16MultiArray

from cv_bridge import CvBridge
import cv2 as cv

import getpass
import threading
encoding = ['16UC1', '32FC1']
import time
import math
import os
from yahboomcar_depth.astra_common import *



class mono_Tracker(Node):
    def __init__(self):
        super().__init__('monoIdentify') 

        self.declare_parameter('x1', 290)
        self.declare_parameter('y1', 160)
        self.declare_parameter('x2', 350)
        self.declare_parameter('y2', 200)
        self.x1 = self.get_parameter('x1').get_parameter_value().integer_value
        self.y1 = self.get_parameter('y1').get_parameter_value().integer_value
        self.x2 = self.get_parameter('x2').get_parameter_value().integer_value
        self.y2 = self.get_parameter('y2').get_parameter_value().integer_value

        self.get_logger().info(f'self.x1:={self.x1}')
        self.get_logger().info(f'self.y1:={self.y1}')
        self.get_logger().info(f'self.x2:={self.x2}')
        self.get_logger().info(f'self.y2:={self.y2}')
        
        self.point_pose = (0, 0, 0)
        self.circle = (0, 0, 0)
        self.hsv_range = ()
        self.circle_r = 0
        self.dyn_update = True
        self.select_flags = False
        self.gTracker_state = True
        self.windows_name = 'frame'
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.end = 0

       
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()

        self.y_ = self.x_ = 0
         #create the publisher
        self.pub_cmdVel = self.create_publisher(Twist,'/cmd_vel',10)
        #create the subscriber
        self.depth_image_sub = Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw')
        self.rgb_image_sub = Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/rgb0/image')
        self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub],queue_size=10,slop=0.5)
        self.ts.registerCallback(self.kcfTrack)

        # self.bject_position_sub = self.create_subscription(Int16MultiArray, 'corner_xy',self.click_callback, 1)
        self.start = self.end = 0

        self.finished_pub = self.create_publisher(Bool, "finished", 5)

        #跟随
        self.prev_dist = 0
        self.Joy_active = False
        self.prev_angular = 0
        self.Center_prevx = 0
        self.Center_prevr = 0
        self.prev_time = time.time()

        self.Robot_Run = False
        self.img_flip = False
        self.twist = Twist()


        self.tracker_type = 'KCF'
        self.VideoSwitch = True
        self.img_flip = False

        print("OpenCV Version: ",cv.__version__)
        self.gTracker = Tracker(tracker_type=self.tracker_type)
        self.Track_state = 'init'

        self.pr_time = time.time()
        self.circle_r = 0 
        self.cur_distance = 0.0
        self.corner_x = self.corner_y = 0.0
        self.minDist = 700
        self.encoding = ['16UC1', '32FC1']
        self.linear_PID = (3.0, 0.0, 1.0)
        self.angular_PID = (0.5, 0.0, 2.0)
        self.PID_init()
        self.dist = 0
        self.declare_param()

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.gTracker_state = True
        self.x_ = 0
        self.y_ = 0


    # def click_callback(self, msg):
    #     self.x1 =  msg.data[0] 
    #     self.y1 =  msg.data[1] 
    #     self.x2 =  msg.data[2] 
    #     self.y2 =  msg.data[3] 
    #     self.gTracker_state = True
        
    
    def declare_param(self):
        self.declare_parameter("linear_Kp",10.0)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",0.5)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.declare_parameter("angular_Kp",35.0)
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.declare_parameter("angular_Ki",0.0)
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.declare_parameter("angular_Kd",0.5)
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.declare_parameter("scale",1000)
        self.scale = self.get_parameter('scale').get_parameter_value().integer_value
        self.declare_parameter("minDistance",0.5)
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        
    def get_param(self):
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.scale = self.get_parameter('scale').get_parameter_value().integer_value
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        
        self.linear_PID = (self.linear_Kp, self.linear_Ki, self.linear_Kd)
        self.angular_PID = (self.angular_Kp, self.angular_Ki, self.angular_Kd)
        self.minDist = self.minDistance * 1000
        

    def PID_init(self):
        self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
        self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)

    def kcfTrack(self,color_frame,depth_frame):
        #rgb_image
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'bgr8')
        result_image = np.copy(rgb_image)
        #depth_image
        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, encoding[1])
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)

        result_image = cv.resize(result_image, (640, 480))
        result_image = self.process(result_image)
       
        
        if self.circle_r > 2 :
    
            h_, w_ = depth_image_info.shape[:2]
            points = []
            if 0 <= self.y_ < h_ and 0 <= self.x_ < w_:
                points.append((int(self.y_), int(self.x_)))
            if 0 <= self.y_+1 < h_ and 0 <= self.x_+1 < w_:
                points.append((int(self.y_+1), int(self.x_+1)))
            if 0 <= self.y_-1 < h_ and 0 <= self.x_-1 < w_:
                points.append((int(self.y_-1), int(self.x_-1)))
            valid_depths = []
            for y, x in points:
                depth_val = depth_image_info[y][x]
                if depth_val != 0:
                    valid_depths.append(depth_val)
            if valid_depths:
                dist = int(sum(valid_depths) / len(valid_depths))
            else:
                dist = 0
            
            text = "x:" + str(int(self.x_)) + " y:" + str(int(self.y_))+ " dist:" + str(dist)
            cv.putText(result_image, text,  (30, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            if dist > 0.2:
                self.execute(self.x_,dist)
            self.dist = dist

            self.Center_prevx = self.x_
            self.Center_prevr = self.y_
            self.prev_time = time.time()
        else:
            now_time = time.time()
            elapsed_time = now_time - self.prev_time
            if elapsed_time > 0.5:
                #self.get_logger().info(f'elapsed_time: {elapsed_time}')
                if elapsed_time > 10:
                    self.finished_pub.publish(Bool(data=True))
                    self.prev_time = now_time
                elif elapsed_time > 5:
                    remaining_time = 10 - int(elapsed_time)
                    if remaining_time > 0:
                        self.get_logger().info(str(remaining_time))
                self.execute(320,self.minDist)

        if self.dist == 0:
            self.pub_cmdVel.publish(Twist())
            
        cv.imshow('frame', result_image)
        cv.waitKey(1)

    def execute(self, point_x, dist):
        self.get_param()

        if self.Joy_active == True: return

        linear_x = 0.2*self.linear_pid.compute(dist, self.minDist)
        angular_z = 0.5*self.angular_pid.compute(point_x,320)

        # print("angular_z",angular_z)
        linear_x = max(-0.5, min(linear_x, 0.5))
        angular_z = max(-0.5, min(angular_z, 0.5))

        if abs(dist - self.minDist) < 40: linear_x = 0
        if abs(point_x - 320.0) < 40: angular_z = 0

        # print("linear_x",linear_x)
        
        twist = Twist()

        twist.angular.z = -angular_z * 1.0
        twist.linear.x = linear_x * 1.0
        self.pub_cmdVel.publish(twist)
        self.Robot_Run = True
        self.x_ = 0

    def process(self, rgb_img):

        if self.gTracker_state == True:
            Roi = (self.x1, self.y1,self.x2 - self.x1, self.y2 - self.y1)
            self.get_logger().info(f'roi={Roi}')
            self.gTracker = Tracker(tracker_type=self.tracker_type)
            self.gTracker.initWorking(rgb_img, Roi)
            self.gTracker_state = False

        rgb_img, (targBegin_x, targBegin_y), (targEnd_x, targEnd_y) = self.gTracker.track(rgb_img)
        center_x = targEnd_x / 2 + targBegin_x / 2
        center_y = targEnd_y / 2 + targBegin_y / 2

        self.x_ = center_x
        self.y_ = center_y
        
        width = targEnd_x - targBegin_x
        high = targEnd_y - targBegin_y
        self.circle_r = min(width, high)

        

        return rgb_img

    def cancel(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)
        print("Shutting down this node.")


def main(args=None):
    rclpy.init(args=args)
    kcf_tracker = mono_Tracker()
    try:
        rclpy.spin(kcf_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        kcf_tracker.cancel()


