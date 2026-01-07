#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray,Int32
import numpy as np

encoding = ['16UC1', '32FC1']

class GetDepthInfo(Node):
    def __init__(self):
        super().__init__('get_depth_info')
        self.declare_parameter('x', 1)
        self.declare_parameter('y', 1)
        self.x = self.get_parameter('x').get_parameter_value().integer_value
        self.y = self.get_parameter('y').get_parameter_value().integer_value

        self.get_logger().info(f'self.x:={self.x}')
        self.get_logger().info(f'self.y:={self.y}')
        self.sub = self.create_subscription(Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw', self.topic_callback, 10)
        # self.bject_position_sub = self.create_subscription(Int16MultiArray, 'corner_xy',self.click_callback, 1)
        self.dist_pub = self.create_publisher(Int32, 'dist_topic', 10)

        self.flag = True
        self.depth_bridge = CvBridge()

    def topic_callback(self, msg):
        if self.flag == False:
            return
        depth_image = self.depth_bridge.imgmsg_to_cv2(msg, encoding[1])
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        dist = depth_image_info[self.y+ 5, self.x + 10]

        dist = round(dist, 3)

        # self.get_logger().info(f'dist:=,{dist}')

        if self.flag== True:
            msg = Int32()
            msg.data = int(dist)
            self.get_logger().info(f'dist={dist}mm')
            self.dist_pub.publish(msg)
            self.flag = False




def main(args=None):
    rclpy.init(args=args)
    get_depth_info = GetDepthInfo()
    rclpy.spin(get_depth_info)
    get_depth_info.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
