#!/usr/bin/env python3
# encoding: utf-8
import threading
import numpy as np
import time
import os
import sys
import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
sys.path.append('/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_mediapipe/yahboomcar_mediapipe')
from media_library import *

class HandCtrlArmNode(Node):
    def __init__(self):
        super().__init__('hand_ctrl_arm_node')

        self.hand_detector = HandDetector()
        self.arm_status = True
        self.locking = True
        self.init = True
        self.add_lock = self.remove_lock = 0

        self.event = threading.Event()
        self.event.set()

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'processed_image', 10)

        camera_type = os.getenv('CAMERA_TYPE', 'usb')
        topic_name = '/ascamera_hp60c/camera_publisher/rgb0/image' if camera_type == 'nuwa' else '/usb_cam/image_raw'
        
        self.subscription = self.create_subscription(
            Image,
            topic_name, 
            self.image_callback,
            10)
        
        self.pTime = time.time()

    def process(self, frame):
        frame, lmList, bbox = self.hand_detector.findHands(frame)
        if len(lmList) != 0:
            threading.Thread(target=self.find_hand_threading, args=(lmList, bbox)).start()
        return frame

    def find_hand_threading(self, lmList, bbox):
        fingers = self.hand_detector.fingersUp(lmList)
        angle = self.hand_detector.ThumbTOforefinger(lmList)
        value = np.interp(angle, [0, 70], [185, 20])
        indexX = (bbox[0] + bbox[2]) / 2
        indexY = (bbox[1] + bbox[3]) / 2
        print("index X: %.1f, Y: %.1f" % (indexX, indexY))

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = self.process(frame)
        processed_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_.publish(processed_image_msg)
        cTime = time.time()
        fps = 1 / (cTime - self.pTime)
        self.pTime = cTime

        cv.putText(frame, f'FPS: {int(fps)}', (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    hand_ctrl_arm_node = HandCtrlArmNode()
    rclpy.spin(hand_ctrl_arm_node)
    hand_ctrl_arm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()