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
import pyzbar.pyzbar as pyzbar
from cv_bridge import CvBridge
from Rosmaster_Lib import Rosmaster
print("import done")


class poseTracker(Node):
    def __init__(self,name):
        super().__init__(name)
        
        #create the publisher
        self.servo_angle = ServoControl()
        self.pub_Servo = self.create_publisher(ServoControl, 'Servo', 10)
        self.image_pub = self.create_publisher(Image, '/image', 10)
        # 创建完成信息发布者 / Create a publisher for finished signals
        self.finished_pub = self.create_publisher(Bool, "finished", 5)
        #create the subscriber
        self.sub_image = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        self.Robot_Run = False
        self.img_flip = False
        self.start_beep = True
        
        self.end = 0
        self.encoding = ['8UC3']
        self.pose_detector = PoseDetector()
        self.Center_x = self.Center_y = self.Center_r = 0
        self.prev_time = time.time()
        # pid init
        self.declare_param()
        self.PID_init()
        self.prev_linear_PID = None
        self.get_param()

        self.frame = None
        #self.timer = self.create_timer(0.011, self.on_timer)
        print("init done")
                
    def declare_param(self):
        # PID
        self.declare_parameter("Kp",30.0)
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.declare_parameter("Ki",0.0)
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.declare_parameter("Kd",2.5)
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value

        self.scale = 1000
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
        self.on_timer()

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

            self.frame, pose_points, _= self.pose_detector.pubPosePoint(self.frame)

            if pose_points:  
                sum_x = sum(point[1] for point in pose_points) 
                sum_y = sum(point[2] for point in pose_points)  
                center_x = sum_x / len(pose_points)  
                center_y = sum_y / len(pose_points)
                cv.circle(self.frame, (int(center_x),int(center_y)), 10, (0, 255, 255), -1)
                text = "x:" + str(int(center_x)) + " y:" + str(int(center_y))
                cv.putText(self.frame, text, (20, 55), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                threading.Thread(target=self.execute, args=(center_x, center_y)).start()
            else:
                now_time = time.time()
                elapsed_time = now_time - self.prev_time
                if elapsed_time > 15:
                    self.finished_pub.publish(Bool(data=True))
                    self.prev_time = now_time
                elif elapsed_time > 10:
                    remaining_time = 15 - int(elapsed_time)
                    if remaining_time > 0:
                        self.get_logger().info(str(remaining_time))   
            #img_msg = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
            cv.imshow('frame', self.frame)
            cv.waitKey(1)
            #self.image_pub.publish(img_msg)



    def PID_init(self):
        self.PID_controller = simplePID(
            [0, 0],
            [self.Kp / float(self.scale), self.Kp / float(self.scale)],
            [self.Ki / float(self.scale), self.Ki / float(self.scale)],
            [self.Kd / float(self.scale), self.Kd / float(self.scale)])
            
    def cancel(self):
        cmd1 = "ros2 topic pub -1 /Servo yahboomcar_msgs/msg/ServoControl "
        cmd2 = f'''"{{'s1': {self.init_servos1}, 's2': {self.init_servos2}}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)
        print("Shutting down this node.")

    def execute(self, point_x, point_y):
        position = Position()
        position.anglex = point_x * 1.0
        position.angley = point_y * 1.0
        position.distance = 0.0
        self.get_logger().info(f'point_x: {point_x}, point_x: {point_y}')
        [x_Pid, y_Pid] = self.PID_controller.update([point_x - 320, point_y - 240])
        #print(f"x_Pid {x_Pid}, y_Pid {y_Pid}")
        x_Pid = x_Pid * (abs(x_Pid) <=self.Kp/2.5)
        y_Pid = y_Pid * (abs(y_Pid) <=self.Kp/2.5)
        if self.img_flip == True:
            self.PWMServo_X -= x_Pid
            self.PWMServo_Y -= y_Pid
        else:
            self.PWMServo_X  += x_Pid
            self.PWMServo_Y  += y_Pid

        self.PWMServo_X = int(max(0, min(180, self.PWMServo_X)))
        self.PWMServo_Y = int(max(0, min(100, self.PWMServo_Y)))
        self.servo_angle.s1 = self.PWMServo_X
        self.servo_angle.s2 = self.PWMServo_Y
        self.pub_Servo.publish(self.servo_angle)



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
    pose_Tracker = poseTracker("PoseTracker")
    try:
        rclpy.spin(pose_Tracker)
    except KeyboardInterrupt:
        pass
    finally:
        pose_Tracker.cancel()
        pose_Tracker.destroy_node()