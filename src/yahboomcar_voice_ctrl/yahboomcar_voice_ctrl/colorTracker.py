#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from yahboomcar_msgs.msg import Position, ServoControl
from cv_bridge import CvBridge
#common lib
import os
import threading
import math
import time
from yahboomcar_astra.common.astra_common import *
from Rosmaster_Lib import Rosmaster
print("import done")

class color_Tracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create the publisher
        self.pub_Servo = self.create_publisher(ServoControl, 'Servo', 10)
        self.image_pub = self.create_publisher(Image, '/image', 10)
        # 创建完成信息发布者 / Create a publisher for finished signals
        self.finished_pub = self.create_publisher(Bool, "finished", 5)
        #create the subscriber
        self.sub_image = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.prev_time = time.time()

        self.Robot_Run = False
        self.img_flip = False
        self.start_beep = True
        self.encoding = ['8UC3']

        # hsv
        self.Roi_init = ()
        self.hsv_range = ()
        self.end = 0
        self.circle = (0, 0, 0)
        self.dyn_update = True
        self.Start_state = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'identify'
        self.color = color_follow()
        self.cols, self.rows = 0, 0

        # hsv_text
        self.hsv_text = "/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_voice_ctrl/yahboomcar_voice_ctrl/colorHSV.text"
        self.hsv_lines = []
        try:
            with open(self.hsv_text, 'r') as f:
                self.hsv_lines = [line.strip() for line in f if line.strip()]
                self.get_logger().info(f"Loaded {len(self.hsv_lines)} color profiles")
        except Exception as e:
            self.get_logger().error(f"Failed to load HSV file: {str(e)}")
        self.frame = None
        self.timer = self.create_timer(0.033, self.on_timer)

        # pid 
        self.declare_param()
        self.PID_init()
        self.prev_linear_PID = None
        self.get_param()

        match self.target_color:
            case 1:self.get_logger().info("Red!")
            case 2:self.get_logger().info("Green!")
            case 3:self.get_logger().info("Blue!")
            case 4:self.get_logger().info("Yellow!")

        print("init done")
        
    def declare_param(self):
        self.declare_parameter("target_color", 1)
        self.target_color = self.get_parameter('target_color').get_parameter_value().integer_value
        # pid init
        self.declare_parameter("linear_Kp",12.0)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",0.8)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value

        self.scale = 1000
        # servo init
        self.init_servos1 = self.PWMServo_X = int(os.environ.get("INIT_SERVO_S1"))
        self.init_servos2 = self.PWMServo_Y = int(os.environ.get("INIT_SERVO_S2"))
        self.servo_angle = ServoControl()
        self.servo_angle.s1 = self.init_servos1
        self.servo_angle.s2 = self.init_servos2
        for i in range(3):
            self.pub_Servo.publish(self.servo_angle)


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

        # PID
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value

        current_linear_PID = (self.linear_Kp, self.linear_Ki, self.linear_Kd)
        if current_linear_PID != self.prev_linear_PID:
            self.linear_pid.update_params(
                [self.linear_Kp / float(self.scale), self.linear_Kp / float(self.scale)],
                [self.linear_Ki / float(self.scale), self.linear_Ki / float(self.scale)],
                [self.linear_Kd / float(self.scale), self.linear_Kd / float(self.scale)]
            )
            self.prev_linear_PID = current_linear_PID


    def PID_init(self):
        self.linear_pid = simplePID(
            [0, 0],
            [self.linear_Kp / float(self.scale), self.linear_Kp / float(self.scale)],
            [self.linear_Ki / float(self.scale), self.linear_Ki / float(self.scale)],
            [self.linear_Kd / float(self.scale), self.linear_Kd / float(self.scale)]
        )

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #self.on_timer()

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
            start = time.time()
            fps = 1 / (start - self.end)
            text_fps = "FPS : " + str(int(fps))
            text = "x:" + str(self.Center_x) + " y:" + str(self.Center_y)
            self.end = start
            cv.putText(self.frame, text_fps, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (100, 200, 200), 2)
            cv.putText(self.frame, text, (30, 55), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)

            if len(binary) != 0: 
                #img_msg = self.bridge.cv2_to_imgmsg(ManyImgs(1, ([self.frame, binary])), "bgr8")
                cv.imshow('frame', ManyImgs(1, ([self.frame, binary])))
            else:
                #img_msg = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
                cv.imshow('frame', self.frame)
            cv.waitKey(1)
            #self.image_pub.publish(img_msg)


    def process(self, rgb_img):
        self.get_param()
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []

        if len(self.hsv_range) != 0:
            rgb_img, binary, self.circle = self.color.object_follow(rgb_img, self.hsv_range)
            self.Start_state = True
            if self.circle[2] != 0:
                self.Center_x = self.circle[0]
                self.Center_y = self.circle[1]
                self.Center_r = self.circle[2]
                threading.Thread(target=self.execute, args=(self.circle[0], self.circle[1], self.circle[2])).start()
                if self.circle[2] >= 8:
                    self.prev_time = time.time()
            now_time = time.time()
            if self.circle[2] < 8 and (now_time - self.prev_time > 3):
                if now_time - self.prev_time > 15:
                    self.finished_pub.publish(Bool(data=True))
                    self.prev_time = now_time
                elif now_time - self.prev_time > 10:
                    remaining_time = 15 - int(now_time - self.prev_time)
                    if remaining_time > 0:
                        self.get_logger().info(str(remaining_time))
                self.execute(320, 240)
        else:
            if self.Start_state == True:
                self.Robot_Run = False
                self.Start_state = False
        return rgb_img, binary

    def execute(self, x, y, z=None):
        position = Position()
        position.anglex = x * 1.0
        position.angley = y * 1.0
        if z is not None:
            position.distance = z * 1.0
        self.get_logger().info(f'point_x: {x}, point_x: {y}')
        [x_Pid, y_Pid] = self.linear_pid.update([(x - 320), y - 240])
        #print(f"x_Pid {x_Pid}, y_Pid {y_Pid}")
        x_Pid = x_Pid * (abs(x_Pid) <=self.linear_Kp/2.4)
        y_Pid = y_Pid * (abs(y_Pid) <=self.linear_Kp/2.4)
        if self.img_flip == True:
            self.PWMServo_X -= x_Pid
            self.PWMServo_Y -= y_Pid
        else:
            self.PWMServo_X  += x_Pid
            self.PWMServo_Y  += y_Pid
        #print(f"x_Pid {x_Pid}, y_Pid {y_Pid},self.PWMServo_X {self.PWMServo_X}, self.PWMServo_Y {self.PWMServo_Y}")
        self.PWMServo_X = int(max(0, min(180, self.PWMServo_X)))
        self.PWMServo_Y = int(max(0, min(100, self.PWMServo_Y)))
        self.servo_angle.s1 = self.PWMServo_X
        self.servo_angle.s2 = self.PWMServo_Y
        #print(f"self.servo_angle.s1 {self.servo_angle.s1}, self.servo_angle.s2 {self.servo_angle.s2}")
        self.pub_Servo.publish(self.servo_angle)


    def cancel(self):
        cmd1 = "ros2 topic pub -1 /Servo yahboomcar_msgs/msg/ServoControl "
        cmd2 = f'''"{{'s1': {self.init_servos1}, 's2': {self.init_servos2}}}"'''
        cmd = cmd1 +cmd2
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
    color_tracker = color_Tracker("ColorTracker")
    print("start it")
    try:
        rclpy.spin(color_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        color_tracker.cancel()
        color_tracker.destroy_node()