#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool,UInt16
from sensor_msgs.msg import Image
#common lib
import os
import threading
import math
from yahboomcar_astra.common.media_common import *
from yahboomcar_msgs.msg import Position,ServoControl
from cv_bridge import CvBridge
from Rosmaster_Lib import Rosmaster
print("import done")


class gestureFollow(Node):
    def __init__(self,name):
        super().__init__(name)
        
        #create the publisher
        self.pub_Servo = self.create_publisher(ServoControl, 'Servo', 10)
        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/image', 10)
        # 创建完成信息发布者 / Create a publisher for finished signals
        self.finished_pub = self.create_publisher(Bool, "finished", 5)
        #create the subscriber
        self.sub_image = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 10)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState',  self.JoyStateCallback,1)
        self.bridge = CvBridge()

        self.Joy_active = False
        self.Robot_Run = False
        self.img_flip = False
        self.start_beep = True
        self.servo_ii = True
        self.twist = Twist()
        self.scale = 1000
        self.end = 0
        self.encoding = ['8UC3']
        self.hand_detector = handDetector(detectorCon=0.8)
        self.Center_x = self.Center_y = self.Center_r = 0
        self.prev_time = time.time()
        # pid init
        self.declare_param()
        self.PID_init()
        self.prev_main_PID = None
        self.prev_angular_PID = None
        self.get_param()
        
        self.frame = None
        self.timer = self.create_timer(0.033, self.on_timer)
        print("init done")
        
    def declare_param(self):
        # limit
        self.declare_parameter("mintra_r",250.0)
        self.mintra_r = self.get_parameter('mintra_r').get_parameter_value().double_value
        self.declare_parameter("tolerance_x",5.0)
        self.tolerance_x = self.get_parameter('tolerance_x').get_parameter_value().double_value
        self.declare_parameter("tolerance_r",50.0)
        self.tolerance_r = self.get_parameter('tolerance_r').get_parameter_value().double_value
        
        # pid init
        self.declare_parameter("linear_Kp",0.5)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",0.5)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value

        self.declare_parameter("angular_Kp",8.0)
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.declare_parameter("angular_Ki",0.0)
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.declare_parameter("angular_Kd",0.5)
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value

        self.scale = 1000
        # servo init
        self.declare_parameter("Servo_Kp",25.0)
        self.Servo_Kp = self.get_parameter('Servo_Kp').get_parameter_value().double_value
        self.declare_parameter("Servo_Ki",0.0)
        self.Servo_Ki = self.get_parameter('Servo_Ki').get_parameter_value().double_value
        self.declare_parameter("Servo_Kd",2.0)
        self.Servo_Kd = self.get_parameter('Servo_Kd').get_parameter_value().double_value
        
        self.init_servos1 = self.PWMServo_X = int(os.environ.get("INIT_SERVO_S1"))
        self.init_servos2 = self.PWMServo_Y = int(os.environ.get("INIT_SERVO_S2"))
        self.servo_angle = ServoControl()
        self.servo_angle.s1 = self.init_servos1
        self.servo_angle.s2 = 60
        for i in range(3):
            self.pub_Servo.publish(self.servo_angle)
            time.sleep(0.01)

    def get_param(self):
        self.mintra_r = self.get_parameter('mintra_r').get_parameter_value().double_value
        self.tolerance_x = self.get_parameter('tolerance_x').get_parameter_value().double_value
        self.tolerance_r = self.get_parameter('tolerance_r').get_parameter_value().double_value
        # PID
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value

        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value

        self.Servo_Kp = self.get_parameter('Servo_Kp').get_parameter_value().double_value
        self.Servo_Ki = self.get_parameter('Servo_Ki').get_parameter_value().double_value
        self.Servo_Kd = self.get_parameter('Servo_Kd').get_parameter_value().double_value

        current_main_PID = (self.linear_Kp, self.linear_Ki, self.linear_Kd, self.Servo_Kp, self.Servo_Ki, self.Servo_Kd)
        if current_main_PID != self.prev_main_PID:
            self.PID_controller.update_params(
                [self.linear_Kp / float(self.scale), self.Servo_Kp / float(self.scale), self.Servo_Kp / float(self.scale)],
                [self.linear_Ki / float(self.scale), self.Servo_Ki / float(self.scale), self.Servo_Ki / float(self.scale)],
                [self.linear_Kd / float(self.scale), self.Servo_Kd / float(self.scale), self.Servo_Kd / float(self.scale)]
            )
            self.prev_main_PID = current_main_PID

        current_angular_PID = (self.angular_Kp, self.angular_Ki, self.angular_Kd)
        if current_angular_PID != self.prev_angular_PID:
            self.angular_PID_controller.update_params(
                [self.angular_Kp / float(self.scale)],
                [self.angular_Ki / float(self.scale)],
                [self.angular_Kd / float(self.scale)]
            )
            self.prev_angular_PID = current_angular_PID

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
            start = time.time()
            fps = 1 / (start - self.end)
            text_fps = "FPS : " + str(int(fps))
            self.end = start
            cv.putText(self.frame, text_fps, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)

            _, _, z=self.hand_detector.findHands(self.frame, draw=False) 
            if len(self.hand_detector.lmList) != 0:
                self.servo_ii = False
                x,y = self.hand_detector.findPoint(9)
                cv.circle(self.frame,(int(x),int(y)),5,(0,255,255),10)
                text = "x:" + str(int(x)) + " y:" + str(int(y))+ " area:" + str(int(z))
                cv.putText(self.frame, text, (20, 55), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                if self.hand_detector.get_gesture() == "Zero":
                    cv.putText(self.frame, "STOP", (300, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
                    self.execute(320, 240,self.mintra_r)
                    self.get_logger().info(f'STOP!')
                else:
                    cv.putText(self.frame, "RUN", (300, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (100, 200, 200), 4)
                    threading.Thread(target=self.execute, args=(x, y, z)).start()
                    self.get_logger().info(f'x: {x}, y: {y}, area: {z}')
                self.prev_time = time.time()
            else:
                now_time = time.time()
                elapsed_time = now_time - self.prev_time
                if elapsed_time > 0.5:
                    #self.get_logger().info(f'elapsed_time: {elapsed_time}')
                    if elapsed_time > 15:
                        self.finished_pub.publish(Bool(data=True))
                        self.prev_time = now_time
                    elif elapsed_time > 10:
                        remaining_time = 15 - int(elapsed_time)
                        if remaining_time > 0:
                            self.get_logger().info(str(remaining_time))
                    self.execute(320, 240,self.mintra_r)
                
            #img_msg = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
            cv.imshow('frame', self.frame)
            cv.waitKey(1)
            #self.image_pub.publish(img_msg)


    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0, 0], 
            [self.linear_Kp / float(self.scale), self.Servo_Kp / float(self.scale), self.Servo_Kp / float(self.scale)],  # 调整参数数量
            [self.linear_Ki / float(self.scale), self.Servo_Ki / float(self.scale), self.Servo_Ki / float(self.scale)],
            [self.linear_Kd / float(self.scale), self.Servo_Kd / float(self.scale), self.Servo_Kd / float(self.scale)]
        )
        
        self.angular_PID_controller = simplePID(
            [0],
            [self.angular_Kp / float(self.scale)],
            [self.angular_Ki / float(self.scale)],
            [self.angular_Kd / float(self.scale)]
        )
            
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        self.servo_angle.s1 = self.PWMServo_X = self.init_servos1
        self.servo_angle.s2 = self.PWMServo_Y = 60     
        self.pub_cmdVel.publish(Twist())

    def cancel(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

        cmd1 = "ros2 topic pub -1 /Servo yahboomcar_msgs/msg/ServoControl "
        cmd2 = f'''"{{'s1': {self.init_servos1}, 's2': {self.init_servos2}}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)
        print("Shutting down this node.")

    def execute(self, x, y, z=None):
        position = Position()
        position.anglex = x * 1.0
        position.angley = y * 1.0
        position.distance = z * 1.0
        if self.Joy_active == True: return
        #print(f"z {z},x {x}")
        [linear_Pid, Servox_Pid, Servoy_Pid] = self.PID_controller.update([z - self.mintra_r, x - 320, y - 240])
        angular_Pid = self.angular_PID_controller.update([self.PWMServo_X - self.init_servos1])[0]
        #print(f"linear_Pid {linear_Pid}, angular_Pid {angular_Pid}")
        linear_Pid = max(-0.3, min(linear_Pid, 0.3)) 
        angular_Pid = max(-3.0, min(angular_Pid, 3.0))

        linear_Pid = abs(linear_Pid) if (z - self.mintra_r > 0) else -abs(linear_Pid)
        angular_Pid = -abs(angular_Pid) if (self.PWMServo_X - self.init_servos1) > 0 else abs(angular_Pid)
        Servox_Pid = Servox_Pid * (abs(Servox_Pid) <=self.Servo_Kp/2)
        Servoy_Pid = Servoy_Pid * (abs(Servoy_Pid) <=self.Servo_Kp/2)
        #print(f"linear_Pid {linear_Pid}, angular_Pid {angular_Pid}, Servoy_Pid {Servoy_Pid}")
        if abs(self.PWMServo_X - self.init_servos1) < self.tolerance_x: angular_Pid = 0.0
        if linear_Pid == 0.0 and Servox_Pid == 0.0 : angular_Pid = 0.0
        if abs(z - self.mintra_r) < self.tolerance_r or z <3: linear_Pid = 0.0
        self.twist.linear.x = -linear_Pid
        if self.img_flip == True:
            self.twist.angular.z = angular_Pid
            self.PWMServo_X -= Servox_Pid
            self.PWMServo_Y -= Servoy_Pid
        else:
            self.twist.angular.z = -angular_Pid
            self.PWMServo_X  += Servox_Pid
            self.PWMServo_Y  += Servoy_Pid
        #print(f"twist.linear.x {self.twist.linear.x}, twist.angular.z {self.twist.angular.z}")
        self.PWMServo_Y = int(max(0, min(100, self.PWMServo_Y)))
        self.PWMServo_X = int(max(0, min(180, self.PWMServo_X)))
        self.servo_angle.s1 = self.PWMServo_X
        self.servo_angle.s2 = self.PWMServo_Y
        if self.servo_ii: self.PWMServo_Y = self.servo_angle.s2 = 60
        #print(f"twist.linear.x {self.twist.linear.x}, twist.angular.z {self.twist.angular.z}, servo_angle.s2 {self.servo_angle.s2}")
        self.pub_Servo.publish(self.servo_angle)
        self.pub_cmdVel.publish(self.twist)


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
        #rospy.loginfo('P:{}, I:{}, D:{}'.format(P, I, D))
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
    gesture_Follow = gestureFollow("GestureFollow")
    try:
        rclpy.spin(gesture_Follow)
    except KeyboardInterrupt:
        pass
    finally:
        gesture_Follow.cancel()
        gesture_Follow.destroy_node()