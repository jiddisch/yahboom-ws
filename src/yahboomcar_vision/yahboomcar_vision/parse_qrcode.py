#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
import pyzbar.pyzbar as pyzbar
from PIL import Image as PILImage, ImageDraw, ImageFont
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge
class QRCodeDecoderNode(Node):
    def __init__(self):
        super().__init__('qr_code_decoder_node')
        self.declare_parameter('font_path', '/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_vision/config/Block_Simplified.TTF')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        self.font_path = self.get_parameter('font_path').get_parameter_value().string_value
        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.bridge = CvBridge()
        camera_type = os.getenv('CAMERA_TYPE', 'usb')
        topic_name = '/ascamera_hp60c/camera_publisher/rgb0/image' if camera_type == 'nuwa' else '/usb_cam/image_raw'
        
        self.subscription = self.create_subscription(
            Image,
            topic_name, 
            self.image_callback,
            10)

    def add_logo(self, img, logo_path):
        # Add logo, open logo image
        icon = PILImage.open(logo_path)
        img_w, img_h = img.size

        # Set the size of the logo
        factor = 6
        size_w = int(img_w / factor)
        size_h = int(img_h / factor)
        icon_w, icon_h = icon.size

        if icon_w > size_w:
            icon_w = size_w
        if icon_h > size_h:
            icon_h = size_h

        # Resize the logo
        icon = icon.resize((icon_w, icon_h), PILImage.Resampling.LANCZOS)

        # Center the logo
        w = int((img_w - icon_w) / 2)
        h = int((img_h - icon_h) / 2)

        # Paste the logo
        img.paste(icon, (w, h), mask=None)

        return img

    def decode_display(self, image):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        # The output Chinese characters need to be converted to Unicode encoding first
        barcodes = pyzbar.decode(gray)
        for barcode in barcodes:
            # Extract the position of the boundary box of the TWO-DIMENSIONAL code
            (x, y, w, h) = barcode.rect
            cv.rectangle(image, (x, y), (x + w, y + h), (225, 0, 0), 5)
            encoding = 'UTF-8'
            # to draw it, you need to convert it to a string
            barcodeData = barcode.data.decode(encoding)
            barcodeType = barcode.type
            # Draw the data and type on the image
            pilimg = PILImage.fromarray(image)
            # create brush
            draw = ImageDraw.Draw(pilimg)  # Print on picture
            # parameter 1: font file path, parameter 2: font size
            fontStyle = ImageFont.truetype(self.font_path, size=12, encoding=encoding)
            # Parameter 1: print coordinates, parameter 2: text, parameter 3: font color, parameter 4: font
            draw.text((x, y - 25), str(barcode.data, encoding), fill=(255, 0, 0), font=fontStyle)
            # PIL picture to CV2 picture
            image = cv.cvtColor(np.array(pilimg), cv.COLOR_RGB2BGR)
            # Print barcode data and barcode type to terminal
            self.get_logger().info(f"Found {barcodeType} barcode: {barcodeData}")
        return image

    def image_callback(self, msg):
        start = time.time()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        frame = self.decode_display(frame)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        cv.imshow('frame', frame)
        if cv.waitKey(10) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDecoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()