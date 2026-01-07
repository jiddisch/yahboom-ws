#!/usr/bin/env python3
# encoding: utf-8

import mediapipe as mp
import cv2 as cv
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import numpy as np
class Objectron(Node):
    def __init__(self, staticMode=False, maxObjects=5, minDetectionCon=0.5, minTrackingCon=0.99):
        super().__init__('objectron')
        self.publisher_ = self.create_publisher(Image, 'objectron_detected', 10)
        self.bridge = CvBridge()
        self.staticMode = staticMode
        self.maxObjects = maxObjects
        self.minDetectionCon = minDetectionCon
        self.minTrackingCon = minTrackingCon
        self.index = 0
        self.modelNames = ['Shoe', 'Chair', 'Cup', 'Camera']
        self.mpObjectron = mp.solutions.objectron
        self.mpDraw = mp.solutions.drawing_utils
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon, self.minTrackingCon, self.modelNames[self.index])
        
        camera_type = os.getenv('CAMERA_TYPE', 'usb')
        topic_name = '/ascamera_hp60c/camera_publisher/rgb0/image' if camera_type == 'nuwa' else '/usb_cam/image_raw'
        
        self.subscription = self.create_subscription(
            Image,
            topic_name, 
            self.image_callback,
            10)
        
        self.pTime = time.time()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Check for key press to switch model or quit
        action = cv.waitKey(1) & 0xFF
        if action == ord('q'):
            self.capture.release()
            cv.destroyAllWindows()
            rclpy.shutdown()
            return
        if action == ord('f') or action == ord('F'):
            self.configUP()

        frame = self.findObjectron(frame)
        
        # Calculate FPS
        cTime = time.time()
        fps = 1 / (cTime - self.pTime)
        self.pTime = cTime
        
        # Display FPS on frame
        cv.putText(frame, f'FPS: {int(fps)}', (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_.publish(msg)

        # Show the frame with object detection
        cv.imshow('Objectron', frame)

    def findObjectron(self, frame):
        cv.putText(frame, self.modelNames[self.index], (int(frame.shape[1] / 2) - 30, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.mpobjectron.process(img_RGB)
        if results.detected_objects:
            for id, detection in enumerate(results.detected_objects):
                self.mpDraw.draw_landmarks(frame, detection.landmarks_2d, self.mpObjectron.BOX_CONNECTIONS)
                self.mpDraw.draw_axis(frame, detection.rotation, detection.translation)
        return frame

    def configUP(self):
        self.index += 1
        if self.index >= len(self.modelNames):
            self.index = 0
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon, self.minTrackingCon, self.modelNames[self.index])

def main(args=None):
    rclpy.init(args=args)
    objectron = Objectron()
    rclpy.spin(objectron)
    objectron.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()