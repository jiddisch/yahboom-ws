#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image
#common lib
import os
import threading
import math
from yahboomcar_astra.common.follow_common import *
from yahboomcar_msgs.msg import ServoControl
from std_msgs.msg import Int32, Bool,UInt16
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from Rosmaster_Lib import Rosmaster
RAD2DEG = 180 / math.pi
print ("import finish")
cv_edition = cv.__version__
print("cv_edition: ",cv_edition)
class LineDetect(Node):
	def __init__(self,name):
		super().__init__(name)
		#create a publisher
		self.qos_profile = QoSProfile(depth=1)
		self.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
		self.pub_cmdVel = self.create_publisher(Twist,"/cmd_vel",1)
		self.pub_Buzzer = self.create_publisher(Bool,"/Buzzer",1)
		self.image_pub = self.create_publisher(Image, '/image', 10)
		# 创建完成信息发布者 / Create a publisher for finished signals
		self.finished_pub = self.create_publisher(Bool, "finished", 5)
		#create the subscriber
		self.sub_image = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 10)
		self.sub_JoyState = self.create_subscription(Bool,"/JoyState",self.JoyStateCallback,1)
		self.sub_scan = self.create_subscription(LaserScan,"/scan",self.registerScan,self.qos_profile)
		self.Joy_active = False
		self.bridge = CvBridge()
		self.circle = ()
		self.hsv_range = ()
		self.Roi_init = ()   
		self.warning = 1
		self.end = 0
		self.Start_state = True
		self.Buzzer_state = False
		self.cols, self.rows = 0, 0
		self.Mouse_XY = (0, 0)

		# hsv_text
		self.hsv_text = "/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_voice_ctrl/yahboomcar_voice_ctrl/colorfollowHSV.text"
		self.hsv_lines = []
		try:
			with open(self.hsv_text, 'r') as f:
				self.hsv_lines = [line.strip() for line in f if line.strip()]
				self.get_logger().info(f"Loaded {len(self.hsv_lines)} color profiles")
		except Exception as e:
			self.get_logger().error(f"Failed to load HSV file: {str(e)}")
		self.color = color_follow()
		self.scale = 1000
		self.declare_param()
		self.PID_init()	
		
		self.img_flip = False
		self.start_beep = True
		self.frame = None
		self.prev_time = time.time()
		self.timer = self.create_timer(0.025, self.on_timer)

		match self.target_color:
			case 1:self.get_logger().info("Red!")
			case 2:self.get_logger().info("Green!")
			case 3:self.get_logger().info("Blue!")
			case 4:self.get_logger().info("Yellow!")

	def declare_param(self):
		self.declare_parameter("target_color", 1)
		self.target_color = self.get_parameter('target_color').get_parameter_value().integer_value
        #PID
		self.declare_parameter("Kp",50)
		self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
		self.declare_parameter("Ki",0)
		self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
		self.declare_parameter("Kd",20)
		self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
		self.FollowLinePID = (self.Kp,self.Ki,self.Kd)
		#other
		self.declare_parameter("scale",1000)
		self.scale = self.get_parameter('scale').get_parameter_value().integer_value
		self.declare_parameter("LaserAngle",30)
		self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().integer_value
		self.declare_parameter("linear",0.25)
		self.linear = self.get_parameter('linear').get_parameter_value().double_value
		self.declare_parameter("ResponseDist",0.45)
		self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
		# servo init
		self.init_servos1 = self.PWMServo_X = int(os.environ.get("INIT_SERVO_S1"))
		self.init_servos2 = self.PWMServo_Y = int(os.environ.get("INIT_SERVO_S2"))
		self.servo_angle = ServoControl()
		self.bot = Rosmaster()
		self.bot.set_pwm_servo(2, self.init_servos1)
		self.bot.set_pwm_servo(3, 0)
		del self.bot
		

	def PID_init(self):
		self.PID_controller = simplePID(
            [0, 0],
            [self.FollowLinePID[0] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[1] / 1.0 / (self.scale), 0],
            [self.FollowLinePID[2] / 1.0 / (self.scale), 0])

	def timeout(self):
		now_time = time.time()
		if now_time - self.prev_time > 15:
			self.finished_pub.publish(Bool(data=True))
			self.bot = Rosmaster()
			self.bot.set_beep(0)
			del self.bot
			self.prev_time = now_time
		elif now_time - self.prev_time > 10:
			remaining_time = 15 - int(now_time - self.prev_time)
			if remaining_time > 0:
				self.get_logger().info(str(remaining_time))

	def execute(self, point_x, color_radius):
		if self.Joy_active == True:
			b = Bool()
			self.Buzzer_state = False
			b.data = False
			self.pub_Buzzer.publish(b)
			if self.Start_state == True:
				self.PID_init()
				self.Start_state = False
			return
		self.Start_state = True
						
		if color_radius < 10:
			print("Not Found")
			self.pub_cmdVel.publish(Twist())
			self.timeout()
		else:
			twist = Twist()
			b = Bool()
			[z_Pid, _] = self.PID_controller.update([(point_x - 320)*1.0/16, 0])
			if self.img_flip == True: twist.angular.z = -z_Pid #-z_Pid
			else: twist.angular.z = +z_Pid
			twist.linear.x = self.linear
			#print(f"twist.angular.z: {twist.angular.z}")
			if self.warning > 10:
				print("Obstacles ahead !!!")
				self.pub_cmdVel.publish(Twist())
				self.Buzzer_state = True
				b.data = True
				self.pub_Buzzer.publish(b)
				self.timeout()

			else:
				self.prev_time = time.time()
				if self.Buzzer_state == True:
					b.data = False
					for i in range(3): self.pub_Buzzer.publish(b)
					self.Buzzer_state = False
                
				if abs(point_x-320)<40:
					twist.angular.z=0.0
				if self.Joy_active == False:
					self.pub_cmdVel.publish(twist)
				else:
					twist.angular.z=0.0

	def process(self, rgb_img):
		self.get_param()
		binary = []
		rgb_img = cv.resize(rgb_img, (640, 480))

		if len(self.hsv_range) != 0:
			rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
			threading.Thread(target=self.execute, args=(self.circle[0], self.circle[2])).start()
		else:
			if self.Start_state == True:
				#self.pub_cmdVel.publish(Twist())
				self.Start_state = False
		return rgb_img, binary


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
			self.frame, binary = self.process(self.frame)

			start = time.time()
			fps = 1 / (start - self.end)
			text = "FPS : " + str(int(fps))
			self.end = start
			cv.putText(self.frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
			if len(binary) != 0: 
				#img_msg = self.bridge.cv2_to_imgmsg(ManyImgs(1, ([self.frame, binary])), "bgr8")
				cv.imshow('frame', ManyImgs(1, ([self.frame, binary])))
			else:
				#img_msg = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
				cv.imshow('frame', self.frame)
			cv.waitKey(1)
			#self.image_pub.publish(img_msg)

        
		
	def JoyStateCallback(self,msg):
		if not isinstance(msg, Bool): return
		self.Joy_active = msg.data
		#self.pub_cmdVel.publish(Twist())

	def registerScan(self, scan_data):
		
		self.warning = 1
		if not isinstance(scan_data, LaserScan): return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
		ranges = np.array(scan_data.ranges)
        # 按距离排序以检查从较近的点到较远的点是否是真实的东西
        # if we already have a last scan to compare to:
		for i in range(len(ranges)):
			angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
            # 通过清除不需要的扇区的数据来保留有效的数据
			if abs(angle) > 180 - self.LaserAngle:
				if ranges[i] > 0.2:
					if ranges[i] < self.ResponseDist: self.warning += 1

	def cancel(self):
		cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
		cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
		cmd = cmd1 +cmd2
		os.system(cmd)

		cmd1 = "ros2 topic pub -1 /Servo yahboomcar_msgs/msg/ServoControl "
		cmd2 = f'''"{{'s1': {self.init_servos1}, 's2': {self.init_servos2}}}"'''
		cmd = cmd1 +cmd2
		os.system(cmd)

		cmd1 = "ros2 topic pub -1 /Buzzer std_msgs/msg/Bool "
		cmd2 = '''"{data: 0}"'''
		cmd = cmd1 +cmd2
		os.system(cmd)
		print("Shutting down this node.")

	def get_param(self):
		self.target_color = self.get_parameter('target_color').get_parameter_value().integer_value
		
		if 1 <= self.target_color <= len(self.hsv_lines):
			hsv_values = list(map(int, self.hsv_lines[self.target_color-1].split(',')))
			if len(hsv_values) == 6:
				self.hsv_range = ((hsv_values[0], hsv_values[1], hsv_values[2]),
								(hsv_values[3], hsv_values[4], hsv_values[5]))
			else:
				self.get_logger().error("Invalid HSV format in file")
		else:
			self.get_logger().error(f"Invalid color index: {self.target_color}")
		#kpi
		self.Kd = self.get_parameter('Kd').get_parameter_value().integer_value
		self.Ki = self.get_parameter('Ki').get_parameter_value().integer_value
		self.Kp = self.get_parameter('Kp').get_parameter_value().integer_value
		self.FollowLinePID = (self.Kp,self.Ki,self.Kd)
		#
		self.scale = self.get_parameter('scale').get_parameter_value().integer_value
		self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().integer_value
		self.linear = self.get_parameter('linear').get_parameter_value().double_value
		self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value

		
def main():
	rclpy.init()
	linedetect = LineDetect("follow_line")
	print("start it")
	try:
		rclpy.spin(linedetect)
	except KeyboardInterrupt:
		pass
	finally:
		linedetect.cancel()
		linedetect.destroy_node()

	
	
	

