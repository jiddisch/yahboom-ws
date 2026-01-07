import re
import rclpy
import subprocess
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from sensor_msgs.msg import Image
import tkinter as tk
from PIL import ImageTk, Image as PILImage
import time
import pygame
from cv_bridge import CvBridge
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int16MultiArray,Bool
from interfaces.action import Rot
from interfaces.srv import Audio2
import math
import yaml
from concurrent.futures import Future
import psutil
from ament_index_python.packages import get_package_share_directory
import os
from threading import Thread
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from utils import large_model_interface
import threading
from rclpy.executors import MultiThreadedExecutor
import cv2
from yahboomcar_msgs.msg import ServoControl
from Rosmaster_Lib import Rosmaster
class CustomActionServer(Node):
    def __init__(self):
        super().__init__('action_service_ndoe')
        #初始化参数配置
        self.init_param_config()
        #初始化ROS通信
        self.init_ros_comunication()
        #加载地图映射文件
        self.load_target_points()
        # 初始化语音合成功能 / Initialize text-to-speech synthesis function
        self.system_sound_init()
        # 初始化语言设置/Initialize language settings
        self.init_language()    
        self.get_logger().info("action service started...")

    def init_param_config(self):
        """
        初始化参数配置 / Initialize parameter configuration
        """
        # 设置夹取启动文件路径 / Set the path for the grasping startup file
        pkg_share = get_package_share_directory("largemodel")
        self.map_mapping_config = os.path.join(pkg_share, "config", "map_mapping.yaml")
        # 声明参数 / Declare parameters
        self.declare_parameter("Servo_topic", "/Servo")
        self.declare_parameter("Speed_topic", "/cmd_vel")
        self.declare_parameter("use_double_llm", False)
        self.declare_parameter("text_chat_mode", False)
        self.declare_parameter("useolinetts", False)
        self.declare_parameter("language", "zh")
        self.declare_parameter("image_topic", "/usb_cam/image_raw")
        self.declare_parameter("regional_setting", "China")

        self.apriltagTracker_future = Future()
        self.faceTracker_future = Future()
        self.gestureTracker_future = Future()
        self.poseTracker_future = Future()
        self.qrTracker_future = Future()
        self.colorTracker_future = Future()
        self.monoTracker_future = Future()

        self.apriltagFollow_future = Future()
        self.faceFollow_future = Future()
        self.gestureFollow_future = Future()
        self.poseFollow_future = Future()
        self.qrFollow_future = Future()
        self.colorFollow_future = Future()
        self.follow_line_future = Future()

        self.get_result_future = Future()
        self.run_flag = False
        # 获取参数值 / Get parameter values
        self.Servo_topic = (
            self.get_parameter("Servo_topic").get_parameter_value().string_value
        )
        self.Speed_topic = (
            self.get_parameter("Speed_topic").get_parameter_value().string_value
        )
        self.use_double_llm = (
            self.get_parameter("use_double_llm").get_parameter_value().bool_value
        )
        self.text_chat_mode = (
            self.get_parameter("text_chat_mode").get_parameter_value().bool_value
        )
        self.useolinetts = (
            self.get_parameter("useolinetts").get_parameter_value().bool_value
        )
        self.language = (
            self.get_parameter("language").get_parameter_value().string_value
        )
        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.regional_setting = (
            self.get_parameter("regional_setting").get_parameter_value().string_value
        )
        self.pkg_path = get_package_share_directory("largemodel")
        self.image_save_path = os.path.join(
            self.pkg_path, "resources_file", "image.png"
        )
        self.current_pose = PoseWithCovarianceStamped()
        self.record_pose = PoseStamped()
        self.combination_mode = False  # 组合模式 / Combination mode
        self.interrupt_flag = False  # 打断标志 / Interrupt flag
        self.action_runing = False  # 动作执行状态 / Action execution status
        self.is_recording = False #录音状态 / Recording status
        self.IS_SAVING = False #是否正在保存图像 /Is saving image
        self.first_record = True  # 首次记录位置 / First record
        # 图像处理对象 / Image processing object
        self.image_msg = None
        self.bridge = CvBridge()
        # 创建模型接口客户端 / Create model interface client
        self.model_client = large_model_interface.model_interface()
        # 打断正在导航标志
        self.nav_runing = False 
        self.nav_status = False

    def init_ros_comunication(self):
        """
        初始化创建ros通信对象、函数 / Initialize creation of ROS communication objects and functions
        """

        #创建舵机话题发布者
        self.pub_Servo = self.create_publisher(ServoControl, self.Servo_topic, 10)
        self.servo_angle = ServoControl()
        self.servo_angle_temp = 0
        self.servo_angle.s1 = self.init_servos1 = int(os.environ.get("INIT_SERVO_S1"))
        self.servo_angle.s2 = self.init_servos2 = int(os.environ.get("INIT_SERVO_S2"))
        bot = Rosmaster()
        bot.set_pwm_servo(2, self.init_servos1)
        bot.set_pwm_servo(3, self.init_servos2)
        del bot
        # 创建速度话题发布者 / Create velocity topic publisher
        self.publisher = self.create_publisher(Twist, self.Speed_topic, 10)
        # 创建导航功能客户端，请求导航动作服务器 / Create navigation function client, request navigation action server
        self.navclient = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # 创建动作执行服务器，用于接受动作列表，并执行动作 / Create action execution server to accept action lists and execute actions
        self._action_server = ActionServer(
            self, Rot, "action_service", self.execute_callback
        )
        # 创建执行动作状态发布者 / Create action execution status publisher
        self.actionstatus_pub = self.create_publisher(String, "actionstatus", 3)
        # 创建发布者，发布 seewhat_handle 话题 / Create publisher to publish seewhat_handle topic
        self.seewhat_handle_pub = self.create_publisher(String, "seewhat_handle", 1)
        # 创建JoyCb话题发布者，启动KCF_Tracker_ALM节点测距的功能 / Create JoyCb topic publisher to enable distance measurement functionality of KCF_Tracker_ALM node
        self.joy_pub = self.create_publisher(Bool, "JoyState", 1)
        # 创建发布者，发布 tts_topic 主题 / Create publisher to publish tts_topic topic
        self.TTS_publisher = self.create_publisher(String, "tts_topic", 5)
        # 创建tf监听者，监听坐标变换 / Create tf listener to monitor coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 创建打断状态发布者 / Create interrupt status publisher
        self.interrupt_flag_pub = self.create_publisher(Bool, "interrupt_flag", 1)
        # wakeup话题订阅者 / Subscribe to wakeup topic
        self.wakeup_sub = self.create_subscription(
            Bool, "wakeup", self.wakeup_callback, 5
        )
        # 图像话题订阅者 / Image topic subscriber
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 1
        )
        # finish话题订阅者 / Subscribe to finish topic
        self.finished_sub = self.create_subscription(
            Bool, "finished", self.finished_callback, 5
        )
        #录音状态话题订阅者 / Record status topic subscriber
        self.record_status_sub=self.create_subscription(Bool, "record_status", self.record_status_callback, 5)
        
    def system_sound_init(
        self,
    ):  # 初始化系统声音相关的功能 / Initialize system sound-related functions
        cartype = os.environ.get("CARTYPE", "error")
        pkg_path = get_package_share_directory("largemodel")

        if self.regional_setting == "China":  # 如果是中国地区
            if self.useolinetts:
                model_type = "oline"
                self.tts_out_path = os.path.join(
                    pkg_path, "resources_file", "tts_output.mp3"
                )
            else:
                model_type = "local"
                self.tts_out_path = os.path.join(
                    pkg_path, "resources_file", "tts_output.wav"
                )

        elif self.regional_setting == "international":  # 如果是国际地区

            model_type = "XUNFEI_FOR_INTERNATIONAL"
            self.tts_out_path = os.path.join(
                pkg_path, "resources_file", "XUNFEI_TTS.mp3"
            )
        else:
            while True:
                self.get_logger().info()(
                    'Please check the regional_setting parameter in yahboom.yaml file, it should be either "China" or "international".'
                )
                time.sleep(1)

        self.model_client.tts_model_init(
            model_type, self.language
        )  # 初始化语音合成模型 / Initialize TTS model
        # 初始化音频播放器 / Initialize audio player
        pygame.mixer.init()
        self.stop_event = threading.Event()
    
    def init_language(self):
        language_list = ["zh", "en"]
        if self.language not in language_list:
            while True:
                self.get_logger().info(
                    "The language setting is incorrect. Please check the action_service'' language setting in the yahboom.yaml file"
                )
                self.get_logger().info(self.language)
                time.sleep(1)

        self.feedback_largemoel_dict = {
            "zh": {  # 中文 / Chinese
                "navigation_1": "机器人反馈:导航目标{point_name}被拒绝",
                "navigation_2": "机器人反馈:执行navigation({point_name})完成",
                "navigation_3": "机器人反馈:执行navigation({point_name})失败，目标点不存在",
                "navigation_4": "机器人反馈:执行navigation({point_name})失败",
                "navigation_5": "机器人反馈:执行navigation({point_name})取消",
                "get_current_pose_success": "机器人反馈:get_current_pose()成功",
                "servo1_move_done": "机器人反馈:执行servo1_move({angular})完成",
                "servo2_move_done": "机器人反馈:执行servo2_move({angular})完成",
                "servo_init_done": "机器人反馈:执行servo_init()完成",
                "servo_shake_done": "机器人反馈:执行servo_shake()完成",
                "servo_nod_done": "机器人反馈:servo_nod()完成",
                "drift_done": "机器人反馈:执行drift()完成",
                "wait_done": "机器人反馈:执行wait({duration})完成",
                "track_done": "机器人反馈:执行追踪任务完成",
                "set_cmdvel_done": "机器人反馈:执行set_cmdvel({linear_x},{linear_y},{angular_z},{duration})完成",
                "move_left_done": "机器人反馈:执行move_left({angle},{angular_speed})完成",
                "move_right_done": "机器人反馈:执行move_right({angle},{angular_speed})完成",
                "dance_done": "机器人反馈:执行dance()完成",
                "response_done": "机器人反馈：回复用户完成",
                "failure_execute_action_function_not_exists": "机器人反馈:动作函数不存在，无法执行",
                "finish": "finish",
                "multiple_done": "机器人反馈：执行{actions}完成",
            },
            "en": {  # 英文 / English
                "navigation_1": "Robot feedback: Navigation target {point_name} rejected",
                "navigation_2": "Robot feedback: Execute navigation({point_name}) completed",
                "navigation_3": "Robot feedback: Execute navigation({point_name}) failed, target does not exist",
                "navigation_4": "Robot feedback: Execute navigation({point_name}) failed",
                "navigation_5": "Robot feedback: Execute navigation({point_name}) cancelled",
                "get_current_pose_success": "Robot feedback: get_current_pose() succeeded",
                "servo1_move_done": "Robot feedback: Execute servo1_move({angular}) completed",
                "servo2_move_done": "Robot feedback: Execute servo2_move({angular}) completed",
                "servo_init_done": "Robot feedback: Execute servo_init() completed",
                "servo_shake_done": "Robot feedback: Execute servo_shake() completed",
                "servo_nod_done": "Robot feedback:servo_nod() completed",
                "drift_done": "Robot feedback: Execute drift() completed",
                "wait_done": "Robot feedback: Execute wait({duration}) completed",
                "track_done": "Robot feedback: Execute tracking task completed",
                "set_cmdvel_done": "Robot feedback: Execute set_cmdvel({linear_x},{linear_y},{angular_z},{duration}) completed",
                "move_left_done": "Robot feedback: Execute move_left({angle},{angular_speed}) completed",
                "move_right_done": "Robot feedback: Execute move_right({angle},{angular_speed}) completed",
                "dance_done": "Robot feedback: Execute dance() completed",
                "response_done": "Robot feedback: Reply to user completed",
                "failure_execute_action_function_not_exists": "Robot feedback: Execute action function not exists",
                "finish": "finish",
                "multiple_done": "Robot feedback: Execution {actions} completed",
            },
        }

    def load_target_points(self):
        """
        加载地图映射文件 /Load map mapping file
        """
        with open(self.map_mapping_config, "r") as file:
            target_points = yaml.safe_load(file)
        self.navpose_dict = {}
        for name, data in target_points.items():
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = data["position"]["x"]
            pose.pose.position.y = data["position"]["y"]
            pose.pose.position.z = data["position"]["z"]
            pose.pose.orientation.x = data["orientation"]["x"]
            pose.pose.orientation.y = data["orientation"]["y"]
            pose.pose.orientation.z = data["orientation"]["z"]
            pose.pose.orientation.w = data["orientation"]["w"]
            self.navpose_dict[name] = pose   

    def record_status_callback(self, msg):
        # self.get_logger().info(f"record_status_callback:{msg.data}")
        if msg.data:
            self.is_recording = True
        else:
            self.is_recording = False

    def wakeup_callback(self,msg): 
        if msg.data:
            if pygame.mixer.music.get_busy():
                self.stop_event.set()    
            if self.action_runing: 
                self.interrupt_flag=True 
                self.get_logger().info("打断动作")       
                self.stop_track()
                #self.stop()
            if self.nav_runing: 
                self.nav_runing = False
                self.nav_status = True
                self.cancel_goal_async()

    def cancel_goal_async(self):
        cmd1 = "ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal"
        cmd2 = ''' "{}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

    def finished_callback(self,msg): 
        if msg.data:     
            self.stop_track()
            #self.cancel()

    def get_current_pose(self):  # 记录当前坐标到record_pose中
        """
        获取当前在全局地图坐标系下的位置 /Get the current position in the global map coordinate system
        """
        # 获取当前目标点坐标
        transform = self.tf_buffer.lookup_transform(
            "map", "base_footprint", rclpy.time.Time()
        )
        # 提取位置和姿态
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = transform.transform.rotation
        self.navpose_dict["zero"] = pose
        # 打印记录的坐标
        position = pose.pose.position
        orientation = pose.pose.orientation
        self.get_logger().info(
            f"Recorded Pose - Position: x={position.x}, y={position.y},\
                                z={position.z},Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}"
        )
        if not self.interrupt_flag:
            self.action_status_pub("get_current_pose_success")   
    
    def action_status_pub(self, key, **kwargs):
        """
        多语言版本的动作结果发布方法
        :param key: 文本标识
        :param**kwargs: 占位符参数
        """
        text_template = self.feedback_largemoel_dict[self.language].get(key)

        try:
            message = text_template.format(**kwargs)
        except KeyError as e:
            self.get_logger().error(f"Translation placeholder error: {e} (key: {key})")
            message = f"[Translation failed: {key}]"

        # 发布消息
        self.actionstatus_pub.publish(String(data=message))
        self.get_logger().info(f"Published message: {message}")

    def navigation(self, point_name):
        """
        从navpose_dict字典中获取目标点坐标.并导航到目标点
        """
        point_name = point_name.strip("'\"")
        if point_name not in self.navpose_dict:
            self.get_logger().error(
                f"Target point '{point_name}' does not exist in the navigation dictionary."
            )
            self.action_status_pub(
                "navigation_3", point_name=point_name
            )  # 目标点地图映射中不存在
            return
        
        if self.first_record:
            # 出发前记录当前在全局地图中的坐标(只有在每个任务周期的第一次执行时才会记录)/ before starting a new task, record the current pose in the global map
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = transform.transform.rotation
            self.navpose_dict["zero"] = pose
            self.first_record = False

        self.nav_runing = True
        # 获取目标点坐标
        target_pose = self.navpose_dict.get(point_name)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        send_goal_future = self.navclient.send_goal_async(goal_msg)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error("Goal was rejected!")
                self.action_status_pub("navigation_1", point_name=point_name)
                return
            # self.get_logger().info("Goal accepted!")
            get_result_future = goal_handle.get_result_async()

            def result_callback(future_result):
                result = future_result.result()
                # self.get_logger().info(f"Navigation result: {result.status}")#using for debug
                if self.nav_status:
                    self.nav_status = False
                    self.action_status_pub(
                        "navigation_5", point_name=point_name
                    )  # 执行导航取消 /execute_navigation_canceled
                    self.nav_runing = False
                else:
                    if result.status == 4:
                        self.action_status_pub(
                            "navigation_2", point_name=point_name
                        )  # 执行导航成功 /execute navigation success
                    else:
                        self.get_logger().info(
                            f"Navigation failed with status: {result.status}"
                        )
                        self.action_status_pub(
                            "navigation_4", point_name=point_name
                        )  # 执行导航失败 /execute_navigation_failed
                    self.nav_runing = False

            get_result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

    def drift(self):
        """
        漂移动作
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.5
        twist.angular.z = 1.5
        self._execute_action(twist, durationtime=2.0)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("drift_done")

    def wait(self, duration):
        duration = float(duration)
        time.sleep(duration)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("wait_done", duration=duration)     

    def seewhat(self):
        self.save_single_image()
        msg = String(data="seewhat")
        self.seewhat_handle_pub.publish(
            msg
        )  # 归一化，发布seewhat话题，由model_service调用大模型
        
    def set_cmdvel(self,linear_x,linear_y, angular_z,duration):#发布cmd_vel
        # 将参数从字符串转换为浮点数
        linear_x = float(linear_x)
        linear_y = float(linear_y)
        angular_z = float(angular_z)
        duration = float(duration)
        twist = Twist()       
        twist.linear.x =  linear_x*1.15
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self._execute_action(twist,durationtime=duration)
        self.stop()
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "set_cmdvel_done",
                linear_x=linear_x,
                linear_y=linear_y,
                angular_z=angular_z,
                duration=duration,
            )
        
    def move_left(self,angle,angular_speed):  # 左转x度  
        angle=float(angle)
        angular_speed=float(angular_speed)     
        angle_rad = math.radians(angle)# 将角度转换为弧度
        duration = abs(angle_rad*1.25 / angular_speed)
        angular_speed=abs(angular_speed)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self._execute_action(twist,1,duration)
        self.stop()
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "move_left_done",
                angle=angle,
                angular_speed=angular_speed,
            )
                 
    def move_right(self,angle,angular_speed):  # 右转x度 
        angle=float(angle)
        angular_speed=float(angular_speed)
        angle_rad = math.radians(angle)# 将角度转换为弧度
        duration = abs(angle_rad*1.25 / angular_speed)
        angular_speed=-abs(angular_speed)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self._execute_action(twist,1,duration)
        self.stop()
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "move_right_done",
                angle=angle,
                angular_speed=angular_speed,
            )

    def dance(self):  # 跳舞
        actions = [
            {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0, "durationtime": 1.0},
            {"linear_x": -0.2, "linear_y": 0.0, "angular_z": 0.0, "durationtime": 1.0},
            {"linear_x": 0.0, "linear_y": 0.5, "angular_z": 0.0, "durationtime": 1.0},
            {"linear_x": 0.0, "linear_y": -0.5, "angular_z": 0.0, "durationtime": 1.0},
            {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.6, "durationtime": 3.0},
            {"linear_x": 0.0, "linear_y": 0.0, "angular_z": -0.6, "durationtime": 3.0},
            {"linear_x": 0.0, "linear_y": 0.0, "angular_z": -0.6, "durationtime": 3.0},
            {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.6, "durationtime": 3.0},
        ]

        for action in actions:
            if self.interrupt_flag:
                break
            twist = Twist()
            twist.linear.x = action["linear_x"]
            twist.linear.y = action["linear_y"]
            twist.angular.z = action["angular_z"]
            self._execute_action(twist, durationtime=action["durationtime"])

        self.stop()
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("dance_done")

    def stop(self):# 停止
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0        
        twist.angular.z = 0.0
        self._execute_action(twist,1,1)

    def stop_servo(self):# 停止舵机
        self.servo_angle.s1 = self.init_servos1
        self.servo_angle.s2 = self.init_servos2
        for i in range(3):
            self.pub_Servo.publish(self.servo_angle) 
    
    def servo1_move(self,angular): # 1号舵机转到x度
        angular = int(angular)
        self.servo_angle.s1 = int(max(0, min(180, angular)))
        self.pub_Servo.publish(self.servo_angle)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "servo1_move_done",
                angular=angular,
            )

    def servo2_move(self,angular): # 2号舵机转到x度
        angular = int(angular)
        self.servo_angle_temp = self.servo_angle.s2 = int(max(0, min(100, angular)))
        if not 30 <= self.servo_angle.s1 <= 150:
            self.servo_angle.s2 = max(10, self.servo_angle.s2)
        self.pub_Servo.publish(self.servo_angle)
        self.servo_angle.s2 = self.servo_angle_temp
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub(
                "servo2_move_done",
                angular=angular,
            )

    def servo_init(self): # 舵机复位
        self.servo_angle.s1 = self.init_servos1
        self.servo_angle.s2 = self.init_servos2
        self.pub_Servo.publish(self.servo_angle)
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("servo_init_done")

    def servo_shake(self): #舵机摇头30度
        bot = Rosmaster()
        delay = 0.3
        bot.set_pwm_servo(2, self.init_servos1)
        bot.set_pwm_servo(3, self.init_servos2 )
        for _ in range(2):
            bot.set_pwm_servo(2, self.init_servos1 - 30)
            time.sleep(0.3)
            bot.set_pwm_servo(2, self.init_servos1 + 30)
            time.sleep(0.3)
        time.sleep(0.5)
        bot.set_pwm_servo(2, self.init_servos1)
        del bot
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("servo_shake_done")

    def servo_nod(self): #舵机点头30度
        bot = Rosmaster()
        delay = 0.3
        bot.set_pwm_servo(2, self.init_servos1)
        bot.set_pwm_servo(3, self.init_servos2)
        for _ in range(2):
            bot.set_pwm_servo(3, self.init_servos2 - 30)
            time.sleep(0.3)
            bot.set_pwm_servo(3, self.init_servos2 + 30)
            time.sleep(0.3)
        time.sleep(0.5)
        bot.set_pwm_servo(2, self.init_servos1)
        bot.set_pwm_servo(3, self.init_servos2)
        del bot
        if not self.combination_mode and not self.interrupt_flag:
            self.action_status_pub("servo_nod_done")

    def _execute_action(self,twist,num=1, durationtime=3.0):
        for _ in range(num):
            start_time = time.time()
            while (time.time() - start_time) < durationtime:
                if self.interrupt_flag:     
                    self.publisher.publish(twist)
                    return
                self.publisher.publish(twist)
                time.sleep(0.1)

    def cancel(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

        cmd1 = "ros2 topic pub -1 /Servo yahboomcar_msgs/msg/ServoControl "
        cmd2 = f'''"{{'s1': {self.init_servos1}, 's2': {self.init_servos2}}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

    def stop_track(self):
        futures = [
            self.apriltagTracker_future,
            self.faceTracker_future,
            self.gestureTracker_future,
            self.poseTracker_future,
            self.qrTracker_future,
            self.colorTracker_future,
            self.monoTracker_future,

            self.apriltagFollow_future,
            self.faceFollow_future,
            self.gestureFollow_future,
            self.poseFollow_future,
            self.qrFollow_future,
            self.colorFollow_future,
            self.follow_line_future,

            self.get_result_future,
        ]
        for future in futures:
            if not future.done():
                future.set_result(True)
        if self.combination_mode:#是否为组合模式，组合模型需要执行完
            return
        elif self.interrupt_flag:
            return
        else:
            self.action_status_pub("track_done")
        
    def apriltagTracker(self):
        try:
            self.apriltagTracker_future = Future()
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'apriltagTracker'])
            while not self.apriltagTracker_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid')
            self.kill_process_tree(process_1.pid)
            self.cancel()
        except:
            self.get_logger().error('apriltagTracker Startup failure')
            return

    def faceTracker(self):
        try:
            self.faceTracker_future = Future()
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'faceTracker'])
            while not self.faceTracker_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)  
            self.get_logger().info(f'killed process_pid')
            self.kill_process_tree(process_1.pid)
            self.cancel()
        except:
            self.get_logger().error('faceTracker Startup failure')
            return

    def gestureTracker(self):
        try:
            self.gestureTracker_future = Future()
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'gestureTracker'])
            while not self.gestureTracker_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)    
            self.get_logger().info(f'killed process_pid')
            self.kill_process_tree(process_1.pid)
            self.cancel()
        except:
            self.get_logger().error('gestureTracker Startup failure')
            return
    
    def poseTracker(self):
        try:
            self.poseTracker_future = Future()
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'poseTracker'])
            while not self.poseTracker_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)    
            self.get_logger().info(f'killed process_pid')
            self.kill_process_tree(process_1.pid)
            self.cancel()     
        except:
            self.get_logger().error('poseTracker Startup failure')
            return
          
    def qrTracker(self):
        try:
            self.qrTracker_future = Future()
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'qrTracker'])
            while not self.qrTracker_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid')
            self.kill_process_tree(process_1.pid)
            self.cancel()    
        except:
            self.get_logger().error('qrTracker Startup failure')
            return


    def colorTrack(self,color):
        try:
            self.colorTracker_future = Future()
            color = color.strip("'\"")
            if color == 'red':
                target_color = int(1)
            elif color == 'green':
                target_color = int(2)
            elif color == 'blue':
                target_color = int(3)
            elif color == 'yellow':
                target_color = int(4)
            else:
                target_color = int(1)
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'colorTracker','--ros-args','-p',f'target_color:={target_color}'])
            while not self.colorTracker_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid')
            self.kill_process_tree(process_1.pid)
            self.cancel()   
        except:
            self.get_logger().error('colorTrack Startup failure')
            return
            
    def monoTracker(self,x1,y1,x2,y2):
        try:
            self.monoTracker_future = Future()
            x1 = int(x1)
            x2 = int(x2)
            y1 = int(y1)
            y2 = int(y2)
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'monoTracker','--ros-args','-p',f'x1:={x1}','-p',f'y1:={y1}','-p',f'x2:={x2}','-p',f'y2:={y2}'])
            while not self.monoTracker_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid')   
            self.kill_process_tree(process_1.pid)
            self.cancel()
        except:
            self.get_logger().error('monoTracker Startup failure')
            return
        
    def apriltagFollow(self):
        try:
            self.apriltagFollow_future = Future()
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'apriltagFollow'])
            while not self.apriltagFollow_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid') 
            self.kill_process_tree(process_1.pid)
            self.cancel()
        except:
            self.get_logger().error('apriltagFollow Startup failure')
            return
        
    def faceFollow(self):
        try:  
            self.faceFollow_future = Future()
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'faceFollow'])
            while not self.faceFollow_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid') 
            self.kill_process_tree(process_1.pid)
            self.cancel()
        except:
            self.get_logger().error('faceFollow Startup failure')
            return

    def gestureFollow(self):
        try:
            self.gestureFollow_future = Future()
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'gestureFollow'])
            while not self.gestureFollow_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid') 
            self.kill_process_tree(process_1.pid)
            self.cancel()
        except:
            self.get_logger().error('gestureFollow Startup failure')
            return

    def poseFollow(self):
        try:
            self.poseFollow_future = Future()
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'poseFollow'])
            while not self.poseFollow_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid') 
            self.kill_process_tree(process_1.pid)
            self.cancel()
        except:
            self.get_logger().error('poseFollow Startup failure')
            return

    def qrFollow(self):
        try:
            self.qrFollow_future = Future()
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'qrFollow'])
            while not self.qrFollow_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid') 
            self.kill_process_tree(process_1.pid)
            self.cancel()
        except:
            self.get_logger().error('qrFollow Startup failure')
            return

    def colorFollow(self,color):
        try:
            self.colorFollow_future = Future() 
            color = color.strip("'\"")
            if color == 'red':
                target_color = int(1)
            elif color == 'green':
                target_color = int(2)
            elif color == 'blue':
                target_color = int(3)
            elif color == 'yellow':
                target_color = int(4)
            else:
                target_color = int(1)
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'colorFollow','--ros-args','-p',f'target_color:={target_color}'])
            while not self.colorFollow_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid') 
            self.kill_process_tree(process_1.pid)
            self.cancel()     
        except:
            self.get_logger().error('colorFollow Startup failure')
            return

    def follow_line(self,color):
        try:
            self.follow_line_future = Future()
            color = color.strip("'\"")
            if color == 'red':
                target_color = int(1)
            elif color == 'green':
                target_color = int(2)
            elif color == 'blue':
                target_color = int(3)
            elif color == 'yellow':
                target_color = int(4)
            else:
                target_color = int(1)
            process_1 = subprocess.Popen(['ros2', 'run', 'yahboomcar_voice_ctrl', 'follow_line','--ros-args','-p',f'target_color:={target_color}'])
            while not self.follow_line_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid')   
            self.kill_process_tree(process_1.pid)
            self.cancel() 
        except:
            self.get_logger().error('follow_line Startup failure')
            return

    # 核心程序，解析动作列表并执行  # Core program, parse and execute action list
    def execute_callback(self, goal_handle):
        """
        动作执行回调函数：分3种情况：  # Action execution callback function: divided into 3 cases:
        1. 动作列表为空  # 1. Empty action list
        2. 动作列表长度为1  # 2. Action list length is 1
        3. 动作列表长度大于1  # 3. Action list length is greater than 1
        文字交互模式下，不进行语音合成和播放  # In text interaction mode, no voice synthesis or playback is performed
        """
        # self.get_logger().info(f"进入动作回调函数")
        if self.is_recording:
            goal_handle.succeed()
            result = Rot.Result()
            result.success = True
            return result
        feedback_msg = Rot.Feedback()
        actions = goal_handle.request.actions
        self.action_runing = True
        if not actions:  # 动作列表为空  # If the action list is empty
            if not self.text_chat_mode and (
                goal_handle.request.llm_response is not None
                or goal_handle.request.text_response != ""
            ):  # 语音模式，播放对话  # Voice mode, play dialogue
                self.model_client.voice_synthesis(
                    goal_handle.request.llm_response, self.tts_out_path
                )
                self.play_audio(self.tts_out_path, feedback=True)
            else:
                self.action_status_pub("response_done")

        elif len(actions) == 1:  # 动作列表长度为1  # If the action list length is 1

            action = actions[0]
            if not self.text_chat_mode and (
                goal_handle.request.llm_response is not None
                or goal_handle.request.text_response != ""
            ):  # 语音模式，播放对话  # Voice mode, play dialogue
                self.model_client.voice_synthesis(
                    goal_handle.request.llm_response, self.tts_out_path
                )
                self.play_audio(self.tts_out_path)

            match = re.match(r"(\w+)\((.*)\)", action)
            action_name, args_str = match.groups()
            if not hasattr(self, action_name):
                self.get_logger().warning(
                    f"action_service: {action} is invalid action,skip execution"
                )
                self.action_status_pub(
                    "failure_execute_action_function_not_exists"
                )  # Robot feedback: action function does not exist, cannot execute

            else:
                action_name, args_str = match.groups()
                args = [arg.strip() for arg in args_str.split(",")] if args_str else []
                method = getattr(self, action_name)
                method(*args)

            if self.interrupt_flag:
                self.interrupt_flag = False
        else:  # 动作列表长度大于1,使能组合模式  # If the action list length is greater than 1, enable combination mode

            self.combination_mode = True
            if not self.text_chat_mode and (
                goal_handle.request.llm_response is not None
                or goal_handle.request.text_response != ""
            ):  # 语音模式，播放对话  # Voice mode, play dialogue
                self.model_client.voice_synthesis(
                    goal_handle.request.llm_response, self.tts_out_path
                )
                self.play_audio_async(self.tts_out_path)

            for action in actions:
                if self.interrupt_flag:
                    break
                match = re.match(r"(\w+)\((.*)\)", action)
                action_name, args_str = match.groups()
                args = [arg.strip() for arg in args_str.split(",")] if args_str else []

                if not hasattr(self, action_name):
                    self.get_logger().warning(
                        f"action_service: {action} is invalid action，skip execution"  # action_service: {action} is an invalid action, skip execution
                    )
                    self.action_status_pub(
                        "failure_execute_action_function_not_exists"
                    )  # Robot feedback: action function does not exist, cannot execute
                else:
                    method = getattr(self, action_name)
                    method(*args)
                    feedback_msg.status = f"action service execute  {action}  successed"

            if not self.interrupt_flag:
                self.action_status_pub(
                    "multiple_done", actions=actions
                )  # Robot feedback: execution of {actions} completed
            self.combination_mode = (
                False  # 重置组合模式标志位  # Reset combination mode flag
            )
        self.stop()  # 执行完全部动作停止机器人  # Stop the robot after executing all actions
        self.action_runing = False  # 重置运行标志位  # Reset running flag
        self.interrupt_flag = False
        goal_handle.succeed()
        result = Rot.Result()
        result.success = True
        #self.get_logger().info(f"退出动作回调函数")
        return result

    def finish_dialogue(self):  # 发布AI模型结束当前流程标志
        self.first_record = True
        self.is_recording = False  #重置录音标志位  # Reset recording flag
        self.action_status_pub("finish")  # 结束当前任务

    def finishtask(self):
        """
        空操作,不反馈消息，用于结束反馈
        """
        return

    @staticmethod
    # def kill_process_tree(pid):
    #     try:
    #         parent = psutil.Process(pid)
    #         for child in parent.children(recursive=True):
    #             child.kill()
    #         parent.kill()
    #     except psutil.NoSuchProcess:
    #         pass
    def kill_process_tree(pid):
        try:
            parent = psutil.Process(pid)
            children = parent.children(recursive=True)
            
            # 先终止所有子进程
            for child in children:
                try:
                    child.terminate()
                except psutil.NoSuchProcess:
                    pass
            
            # 等待子进程终止
            gone, alive = psutil.wait_procs(children, timeout=3)
            
            # 强制杀死仍然存活的进程
            for p in alive:
                try:
                    p.kill()
                except psutil.NoSuchProcess:
                    pass
            
            # 最后终止父进程
            try:
                parent.terminate()
                parent.wait(timeout=3)
            except psutil.TimeoutExpired:
                parent.kill()
            except psutil.NoSuchProcess:
                pass
                
        except psutil.NoSuchProcess:
            pass

    def play_audio(self, file_path: str, feedback: Bool = False) -> None:
        """
        同步方式播放音频函数The function for playing audio in synchronous mode
        """
        if self.is_recording:
            return
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            if self.stop_event.is_set() or self.is_recording:
                pygame.mixer.music.stop()
                self.stop_event.clear()  # 清除事件
                return
            pygame.time.Clock().tick(10)
        if feedback:
            self.action_status_pub("response_done")

    def play_audio_async(self, file_path: str, feedback: Bool = False) -> None:
        """
        异步方式播放音频函数The function for playing audio in asynchronous mode
        """
        if self.is_recording:
            return
        def target():
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                if self.stop_event.is_set() or self.is_recording:
                    pygame.mixer.music.stop()
                    self.stop_event.clear()  # 清除事件
                    return
                pygame.time.Clock().tick(5)
            if feedback:
                self.action_status_pub("response_done")

        thread = threading.Thread(target=target)
        thread.daemon = True
        thread.start()
        
    def save_single_image(self):
        """
        保存一张图片 / Save a single image
        """
        self.IS_SAVING=True
        time.sleep(0.1)
        if self.image_msg is None:
            self.get_logger().warning("No image received yet.")  # 尚未接收到图像...
            return
        try:
            # 将ROS图像消息转换为OpenCV图像 / Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")
            # 保存图片 / Save the image
            cv2.imwrite(self.image_save_path, cv_image)
            display_thread = threading.Thread(target=self.display_saved_image)
            display_thread.start()

        except Exception as e:
            self.get_logger().error(f"Error saving image: {e}")  # 保存图像时出错...
        self.IS_SAVING=False

    def display_saved_image(self):
        try:
            root = tk.Tk()
            root.title("Image Viewer")    
            img = PILImage.open(self.image_save_path)
            photo = ImageTk.PhotoImage(img)
            label = tk.Label(root, image=photo)
            label.pack()
            root.after(4000, root.destroy)
            root.mainloop()
        except Exception as e:
            print(f"Unable to display image: {e}")

    def image_callback(self, msg):  # 图像回调函数 / Image callback function
        if not self.IS_SAVING:
            self.image_msg = msg
        else:
            self.get_logger().warning("The image is being saved and no new information will be accepted")

def main(args=None):
    rclpy.init(args=args)
    custom_action_server = CustomActionServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(custom_action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        custom_action_server.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

