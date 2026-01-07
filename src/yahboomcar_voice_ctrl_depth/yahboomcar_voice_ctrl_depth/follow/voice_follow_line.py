#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
#common lib
import os
import threading
import math
from yahboomcar_depth.follow_common import *
from std_msgs.msg import Int32, Bool,UInt16
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
RAD2DEG = 180 / math.pi
import re

print ("import finish")
cv_edition = cv.__version__
print("cv_edition: ",cv_edition)

class LineDetect(Node):
	def __init__(self,name):
		super().__init__(name)
		#create a publisher
		self.qos_profile = QoSProfile(depth=1)
		self.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
		#create the publisher
		self.pub_cmdVel = self.create_publisher(Twist,'/cmd_vel',10)
		self.pub_Buzzer = self.create_publisher(Bool,"/Buzzer",1)

		#create a subscriber
		self.sub_JoyState = self.create_subscription(Bool, "/JoyState", self.JoyStateCallback, 1)
		self.sub_scan = self.create_subscription(LaserScan, "/scan", self.registerScan, self.qos_profile)
		self.sub_image = self.create_subscription(Image, "/ascamera_hp60c/camera_publisher/rgb0/image", self.image_callback, self.qos_profile)

		self.image_pub = self.create_publisher(Image,'imags', 500)
		self.finished_pub = self.create_publisher(Bool, "finished", 5)
		#跟随
		self.frame = None
		self.rgb_bridge = CvBridge()
		self.depth_bridge = CvBridge()

		self.Joy_active = False
		self.img = None
		self.circle = ()
		self.hsv_range = ()
		self.Roi_init = ()   
		self.warning = 1
	
		self.Buzzer_state = False
		self.Mouse_XY = (0, 0)

		self.declare_parameter('colcor', 1.0)
		self.colcor_type = int(self.get_parameter('colcor').get_parameter_value().double_value)

		self.hsv_text = "/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_voice_ctrl/yahboomcar_voice_ctrl/colorHSV.text"
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
		self.prev_time = time.time()
		self.get_param()
		self.PID_init()	
		print("Get self.target_id is ",self.colcor_type)   
		self.timer = self.create_timer(0.011, self.on_timer)

	def image_callback(self, msg):
		try:
			cv_img = self.rgb_bridge.imgmsg_to_cv2(msg, "bgr8")
			self.frame = cv_img
		except Exception as e:
			self.get_logger().error(f"CV Bridge error: {e}")


	def declare_param(self):
		#HSV
		self.declare_parameter("Hmin",0)
		self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
		self.declare_parameter("Smin",85)
		self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
		self.declare_parameter("Vmin",126)
		self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
		self.declare_parameter("Hmax",9)
		self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
		self.declare_parameter("Smax",253)
		self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
		self.declare_parameter("Vmax",253)
		self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
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

		
		

	def PID_init(self):
		self.PID_controller = simplePID(
			[0, 0],
			[self.FollowLinePID[0] / 1.0 / (self.scale), 0],
			[self.FollowLinePID[1] / 1.0 / (self.scale), 0],
			[self.FollowLinePID[2] / 1.0 / (self.scale), 0])

	def timeout(self):
		now_time = time.time()
		elapsed_time = now_time - self.prev_time
		if elapsed_time > 10:
			self.finished_pub.publish(Bool(data=True))
			self.prev_time = now_time
			b = Bool()
			b.data = False
			for i in range(3): self.pub_Buzzer.publish(b)

			
		elif elapsed_time > 5:
			remaining_time = 10 - int(elapsed_time)
			if remaining_time > 0:
				self.get_logger().info(str(remaining_time))

	def execute(self, point_x, color_radius):

		if color_radius < 4: 
			print("Not Found")
			self.pub_cmdVel.publish(Twist())
			self.timeout()
			
		else:
			twist = Twist()
			b = Bool()
			[z_Pid, _] = self.PID_controller.update([(point_x - 320)*1.0/16, 0])
			twist.angular.z = +z_Pid
			twist.linear.x = self.linear
			self.get_logger().info(f'x={point_x}, color_radius={color_radius}')
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
		binary = []
		rgb_img = cv.resize(rgb_img, (640, 480))
		
		rgb_img, binary, self.circle = self.color.line_follow(rgb_img, self.hsv_range)
		if len(self.circle) != 0:
			threading.Thread(target=self.execute, args=(self.circle[0], self.circle[2])).start()
		return rgb_img, binary

	def on_timer(self):
		if self.frame is None:
			return
		self.get_param()
		result_image = self.frame.copy()
		
		result_image = cv.resize(result_image, (640, 480))
		result_image, binary= self.process(result_image)

		if len(binary) != 0: 
			cv.imshow('frame', ManyImgs(1, ([result_image, binary])))
		else:
			cv.imshow('frame', result_image)
		cv.waitKey(1)
		
	def get_color_hsv(self, color_name="red"):
		try:
			with open(self.hsv_colcor, 'r') as file:
				content = file.read()
			
			# 正则匹配：匹配 "key": [values] 格式
			pattern = r'"([^"]+)":\s*\[([^\]]+)\]'
			matches = re.findall(pattern, content)
			
			color_dict = {}
			for key, values in matches:
				# 将字符串数值转为整数列表
				values = [int(x.strip()) for x in values.split(',')]
				color_dict[key] = values
			
			return color_dict.get(color_name)
		
		except Exception as e:
			print(f"Error reading HSV file: {e}")
			return None
		
	def JoyStateCallback(self,msg):
		if not isinstance(msg, Bool): return
		self.Joy_active = msg.data


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

		cmd1 = "ros2 topic pub -1 /Buzzer std_msgs/msg/Bool "
		cmd2 = '''"{data: 0}"'''
		cmd = cmd1 +cmd2
		os.system(cmd)
		print("Shutting down this node.")
		cv.destroyAllWindows()
		self.destroy_node()	

	def Reset(self):
		self.PID_init()
	
		self.hsv_range = ()
		self.Joy_active =False
		self.Mouse_XY = (0, 0)
		self.pub_cmdVel.publish(Twist())

		print("Reset succes!!!")

	def get_param(self):

		if 1 <= self.colcor_type <= len(self.hsv_lines):
			hsv_values = list(map(int, self.hsv_lines[self.colcor_type-1].split(',')))
			if len(hsv_values) == 6:
				self.hsv_range = ((hsv_values[0], hsv_values[1], hsv_values[2]),
								(hsv_values[3], hsv_values[4], hsv_values[5]))
			else:
				self.get_logger().error("Invalid HSV format in file")
		else:
			self.get_logger().error(f"Invalid color index: {self.colcor_type}")
		#hsv
		self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
		self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
		self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
		self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
		self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
		self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
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

	
	
	

