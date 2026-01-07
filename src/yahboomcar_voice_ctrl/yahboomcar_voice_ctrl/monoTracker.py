#!/usr/bin/env python3
# encoding: utf-8
import getpass
import threading
import time  # 新增导入 time 模块
from sensor_msgs.msg import CompressedImage, Image
from yahboomcar_msgs.msg import Position, ServoControl
from yahboomcar_astra.common.track_common import *
from std_msgs.msg import Int32, Bool, UInt16
import math
import os
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from Rosmaster_Lib import Rosmaster
class mono_Tracker(Node):
    def __init__(self, name):
        super().__init__(name)
        #create the publisher
        self.pub_Servo = self.create_publisher(ServoControl, 'Servo', 10)
        self.pub_img = self.create_publisher(Image, '/image', 10)
        # 创建完成信息发布者 / Create a publisher for finished signals
        self.finished_pub = self.create_publisher(Bool, "finished", 5)
        #create the subscriber
        self.sub_img = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        self.point_pose = (0, 0, 0)
        self.circle = (0, 0, 0)
        self.dyn_update = True
        self.gTracker_state = False
        self.windows_name = 'frame'

        self.tracker_types = ['BOOSTING', 'MIL', 'KCF']
        self.index = 2
        self.tracker_type = self.tracker_types[self.index]
        self.VideoSwitch = True
        self.img_flip = False
        self.start_beep = True
        self.prev_time = time.time()
        self.scale = 1000

        self.declare_param()
        self.PID_init()
        self.prev_linear_PID = None
        self.get_param()

        print("OpenCV Version: ", cv.__version__)
        self.gTracker = Tracker(tracker_type=self.tracker_type)
        self.Track_state = 'tracking' 

        self.frame = None
        self.timer = self.create_timer(0.03, self.on_timer)

    def declare_param(self):
        # ROI
        self.declare_parameter("x1", 270)
        self.declare_parameter("y1", 190)
        self.declare_parameter("x2", 370)
        self.declare_parameter("y2", 290)
        self.x1 = self.get_parameter('x1').get_parameter_value().integer_value
        self.y1 = self.get_parameter('y1').get_parameter_value().integer_value
        self.x2 = self.get_parameter('x2').get_parameter_value().integer_value
        self.y2 = self.get_parameter('y2').get_parameter_value().integer_value
        #PID
        self.declare_parameter("Kp",15.0)
        self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
        self.declare_parameter("Ki",0)
        self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
        self.declare_parameter("Kd",0.1)
        self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
        # servo init
        self.init_servos1 = self.PWMServo_X = int(os.environ.get("INIT_SERVO_S1"))
        self.init_servos2 = self.PWMServo_Y = int(os.environ.get("INIT_SERVO_S2"))
        self.servo_angle = ServoControl()
        self.servo_angle.s1 = self.init_servos1
        self.servo_angle.s2 = self.init_servos2
        for i in range(3):
            self.pub_Servo.publish(self.servo_angle)
            time.sleep(0.01) 

    def get_param(self):
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value

        current_linear_PID = (self.Kp, self.Ki, self.Kd)
        if current_linear_PID != self.prev_linear_PID:
            self.PID_controller.update_params(
                [self.Kp / float(self.scale), self.Kp / float(self.scale)],
                [self.Ki / float(self.scale), self.Ki / float(self.scale)],
                [self.Kd / float(self.scale), self.Kd / float(self.scale)]
            )
            self.prev_linear_PID = current_linear_PID

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def on_timer(self):
        if self.frame is not None:
            if self.start_beep :
                self.bot = Rosmaster()
                self.bot.set_beep(500)
                del self.bot
                self.start_beep = False
            if self.img_flip:
                self.frame = cv.flip(self.frame, 1)
            self.get_param()

            self.frame, binary = self.process(self.frame)

            #img_msg = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
            cv.imshow('frame', self.frame)
            cv.waitKey(1)
            #self.pub_img.publish(img_msg)

    def process(self, rgb_img):
        binary = []
        if rgb_img is None or rgb_img.size == 0:
            self.get_logger().warning('Received empty image. Skipping processing.')
            return rgb_img, binary

        if self.Track_state == 'tracking':
            if not self.gTracker_state:
                x1 = max(0, self.x1)
                y1 = max(0, self.y1)
                x2 = max(x1 + 1, self.x2)
                y2 = max(y1 + 1, self.y2)
                Roi = (x1, y1, x2 - x1, y2 - y1)

                self.get_logger().info(f"Initializing tracker with ROI: {Roi}")

                self.gTracker.initWorking(rgb_img, Roi)
                self.gTracker_state = True
            try:
                rgb_img, (targBegin_x, targBegin_y), (targEnd_x, targEnd_y) = self.gTracker.track(rgb_img)
                if (targBegin_x < 0 or targBegin_y < 0 or targEnd_x < 0 or targEnd_y < 0):
                    raise ValueError("Invalid tracking result")
                if (targBegin_x == 0 and targBegin_y == 0 and targEnd_x == 0 and targEnd_y == 0):
                    now_time = time.time()
                    elapsed_time = now_time - self.prev_time
                    if elapsed_time > 15:
                        self.finished_pub.publish(Bool(data=True))
                        self.prev_time = now_time
                    elif elapsed_time > 10:
                        remaining_time = 15 - int(elapsed_time)
                        if remaining_time > 0:
                            self.get_logger().info(str(remaining_time))
                else:
                    self.prev_time = time.time()
                center_x = targEnd_x / 2 + targBegin_x / 2
                center_y = targEnd_y / 2 + targBegin_y / 2
                width = targEnd_x - targBegin_x
                high = targEnd_y - targBegin_y
                self.point_pose = (center_x, center_y, min(width, high))
                self.target_lost = False
                threading.Thread(target=self.execute, args=(self.point_pose[0], self.point_pose[1])).start()
            except Exception as e:
                self.get_logger().warning(f"Target lost: {str(e)}")
                self.target_lost = True
        if self.tracker_type in self.tracker_types:
            cv.putText(rgb_img, " Tracker", (260, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        return rgb_img, binary

    def execute(self, point_x, point_y):
        if self.target_lost or point_x == 0 and point_y == 0:
            return
        position = Position()
        position.anglex = point_x * 1.0
        position.angley = point_y * 1.0
        position.distance = 0.0
        self.get_logger().info(f'point_x: {point_x}, point_y: {point_y}')
        [x_Pid, y_Pid] = self.PID_controller.update([point_x - 320, point_y - 240])
        x_Pid = x_Pid * (abs(x_Pid) <=self.Kp/3)
        y_Pid = y_Pid * (abs(y_Pid) <=self.Kp/3)
        if self.img_flip:
            self.PWMServo_X -= x_Pid
            self.PWMServo_Y -= y_Pid
        else:
            self.PWMServo_X += x_Pid
            self.PWMServo_Y += y_Pid

        self.PWMServo_X = int(max(0, min(180, self.PWMServo_X)))
        self.PWMServo_Y = int(max(0, min(100, self.PWMServo_Y)))
        self.servo_angle.s1 = self.PWMServo_X
        self.servo_angle.s2 = self.PWMServo_Y
        self.pub_Servo.publish(self.servo_angle)

    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.Kp / float(self.scale), self.Kp / float(self.scale)],
            [self.Ki / float(self.scale), self.Ki / float(self.scale)],
            [self.Kd / float(self.scale), self.Kd / float(self.scale)]
        )

    def cancel(self):
        cmd1 = "ros2 topic pub -1 /Servo yahboomcar_msgs/msg/ServoControl "
        cmd2 = f'''"{{'s1': {self.init_servos1}, 's2': {self.init_servos2}}}"'''
        cmd = cmd1 + cmd2
        os.system(cmd)
        print("Shutting down this node.")

class simplePID:
    '''very simple discrete PID controller'''

    def __init__(self, target, P, I, D):
        '''Create a discrete PID controller
        each of the parameters may be a vector if they have the same length
        Args:
        target (double) -- the target value(s)
        P, I, D (double)-- the PID parameter
        '''
        # check if parameter shapes are compatabile.
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or (
                np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.last_error = 0
        self.integrator = 0
        self.timeOfLastCall = None
        self.setPoint = np.array(target)
        self.integrator_max = float('inf')

    def update_params(self, P, I, D):
        '''Dynamic update of PID parameters'''
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.integrator = 0
        self.last_error = 0

    def update(self, current_value):
        '''Updates the PID controller.
        Args:
            current_value (double): vector/number of same legth as the target given in the constructor
        Returns:
            controll signal (double): vector of same length as the target
        '''
        current_value = np.array(current_value)
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if (self.timeOfLastCall is None):
            # the PID was called for the first time. we don't know the deltaT yet
            # no controll signal is applied
            self.timeOfLastCall = time.perf_counter()
            return np.zeros(np.size(current_value))
        error = self.setPoint - current_value
        P = error
        currentTime = time.perf_counter()
        deltaT = (currentTime - self.timeOfLastCall)
        # integral of the error is current error * time since last update
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        # derivative is difference in error / time since last update
        D = (error - self.last_error) / deltaT
        self.last_error = error
        self.timeOfLastCall = currentTime
        # return controll signal
        return self.Kp * P + self.Ki * I + self.Kd * D

def main():
    rclpy.init()
    mono_tracker = mono_Tracker("MonoIdentify")
    try:
        rclpy.spin(mono_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        mono_tracker.cancel()
        mono_tracker.destroy_node()

if __name__ == '__main__':
    main()
