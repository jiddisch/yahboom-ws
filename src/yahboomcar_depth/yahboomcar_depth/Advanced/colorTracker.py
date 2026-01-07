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
from yahboomcar_depth.astra_common import *
from cv_bridge import CvBridge
print("import done")

class color_Tracker(Node):
    def __init__(self,name):
        super().__init__(name)
        #create the publisher
        self.pub_cmdVel = self.create_publisher(Twist,'/cmd_vel',10)
        self.declare_param()
        self.PID_init()
        self.get_param()
        
        #create the subscriber
        self.depth_image_sub = Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw')
        self.rgb_image_sub = Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/rgb0/image')
        self.ts = ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub],queue_size=10,slop=0.5)
        self.ts.registerCallback(self.depth_img_Callback)

        self.sub_JoyState = self.create_subscription(Bool,'/JoyState',  self.JoyStateCallback,1)
        self.start = self.end = 0
        #识别
        self.Roi_init = ()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.dyn_update = True
        self.Start_state = False
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.Track_state = 'init'
        self.color = color_detect()
        self.dist = 0

        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.hsv_text = "/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_depth/yahboomcar_depth/colorHSV.text"

        #跟随
        self.rgb_bridge = CvBridge()
        self.depth_bridge = CvBridge()

        self.minDist = 400
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.Center_prevx = 0
        self.Center_prevr = 0
        self.prev_time = 0
        self.prev_dist = 0
        self.prev_angular = 0
        self.Joy_active = False
        self.Robot_Run = False

        self.encoding = ['16UC1', '32FC1']

        print("init done")

        

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
        
    def declare_param(self):
        self.declare_parameter("linear_Kp",0.5)
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.declare_parameter("linear_Ki",0.0)
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.declare_parameter("linear_Kd",1.0)
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.declare_parameter("angular_Kp",1.8)
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.declare_parameter("angular_Ki",0.0)
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.declare_parameter("angular_Kd",0.5)
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.declare_parameter("scale",1000.0)
        self.scale = self.get_parameter('scale').get_parameter_value().double_value
        self.declare_parameter("minDistance",0.5)
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        

        # HSV
        self.declare_parameter("Hmin", 0)
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.declare_parameter("Smin", 85)
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.declare_parameter("Vmin", 126)
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.declare_parameter("Hmax", 9)
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.declare_parameter("Smax", 253)
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.declare_parameter("Vmax", 253)
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value

        self.linear_PID = (self.linear_Kp, self.linear_Ki, self.linear_Kd)
        self.angular_PID = (self.angular_Kp, self.angular_Ki, self.angular_Kd)
        self.minDist = self.minDistance * 1000

    def get_param(self):
        self.linear_Kp = self.get_parameter('linear_Kp').get_parameter_value().double_value
        self.linear_Ki = self.get_parameter('linear_Ki').get_parameter_value().double_value
        self.linear_Kd = self.get_parameter('linear_Kd').get_parameter_value().double_value
        self.angular_Kp = self.get_parameter('angular_Kp').get_parameter_value().double_value
        self.angular_Ki = self.get_parameter('angular_Ki').get_parameter_value().double_value
        self.angular_Kd = self.get_parameter('angular_Kd').get_parameter_value().double_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value
        self.minDistance = self.get_parameter('minDistance').get_parameter_value().double_value
        
        self.linear_pid.update_params(self.linear_Kp/self.scale, self.linear_Ki/self.scale, self.linear_Kd/self.scale)
        self.angular_pid.update_params(self.angular_Kp/self.scale, self.angular_Ki/self.scale, self.angular_Kd/self.scale)
        self.minDist = self.minDistance * 1000

         # HSV 
        self.Hmin = self.get_parameter('Hmin').get_parameter_value().integer_value
        self.Smin = self.get_parameter('Smin').get_parameter_value().integer_value
        self.Vmin = self.get_parameter('Vmin').get_parameter_value().integer_value
        self.Hmax = self.get_parameter('Hmax').get_parameter_value().integer_value
        self.Smax = self.get_parameter('Smax').get_parameter_value().integer_value
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().integer_value
        

    def PID_init(self):
        self.linear_pid = simplePID(self.linear_PID[0] / 1000.0, self.linear_PID[1] / 1000.0, self.linear_PID[2] / 1000.0)
        self.angular_pid = simplePID(self.angular_PID[0] / 100.0, self.angular_PID[1] / 100.0, self.angular_PID[2] / 100.0)
        
    def depth_img_Callback(self,color_frame,depth_frame):
        self.get_param()
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(color_frame,'bgr8')
        result_image = np.copy(rgb_image)

        depth_image = self.depth_bridge.imgmsg_to_cv2(depth_frame, self.encoding[1])
        frame = cv.resize(depth_image, (640, 480))
        depth_image_info = frame.astype(np.float32)
        
        action = cv.waitKey(10) & 0xFF
        result_image = cv.resize(result_image, (640, 480))
        result_frame, binary= self.process(result_image,action)

        self.end = time.time()
        fps = 1 / (self.end - self.start)
        self.start = self.end
        text = "FPS : " + str(int(fps))
        cv.putText(result_frame,text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)

        if self.Center_r > 10 :
            # print(self.Center_r)
            h_, w_ = depth_image_info.shape[:2]
            points = []
            if 0 <= self.Center_y < h_ and 0 <= self.Center_x < w_:
                points.append((int(self.Center_y), int(self.Center_x)))
            if 0 <= self.Center_y+1 < h_ and 0 <= self.Center_x+1 < w_:
                points.append((int(self.Center_y+1), int(self.Center_x+1)))
            if 0 <= self.Center_y-1 < h_ and 0 <= self.Center_x-1 < w_:
                points.append((int(self.Center_y-1), int(self.Center_x-1)))
                
            valid_depths = []
            for y, x in points:
                depth_val = depth_image_info[y][x]
                if depth_val != 0:
                    valid_depths.append(depth_val)
            if valid_depths:
                dist = int(sum(valid_depths) / len(valid_depths))
            else:
                dist = 0
            self.dist = dist

            text = "x:" + str(int(self.Center_x)) + " y:" + str(int(self.Center_y))+ " dist:" + str(dist)
            cv.putText(result_frame, text, (20, 55), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

            if dist > 0.2 and self.Start_state == True:
                self.execute(self.Center_x, dist)

            self.Center_prevx = self.Center_x
            self.Center_prevr = self.Center_r
        else:
            now_time = time.time()
            if now_time - self.prev_time > 0.75:
                self.prev_time = now_time
                self.execute(320,self.minDist)

        if self.dist == 0:
            self.pub_cmdVel.publish(Twist())
        if len(binary) != 0: cv.imshow(self.windows_name, ManyImgs(1, ([result_frame, binary])))
        else:
            cv.imshow(self.windows_name, result_frame)
        
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        self.pub_cmdVel.publish(Twist())

    def cancel(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)
        print("Shutting down this node.")
        cv.destroyAllWindows()
        self.destroy_node()
        
    def cleanup(self):
        self.pub_cmdVel.publish(Twist())
        print ("Shutting down this node.")
        cv.destroyAllWindows()
    def Reset(self):
        self.hsv_range = ()
        self.Center_r = 0
        self.pub_cmdVel.publish(Twist())
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        self.cx = 0
        self.cy = 0

        
    def process(self, rgb_img, action):
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == 32: self.Track_state = 'tracking'
        elif action == ord('i') or action == ord('I'): self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): self.Reset()
        elif action == ord('q') or action == ord('Q'): self.cancel()
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text): 
                self.hsv_range = read_HSV(self.hsv_text)
            else: 
                self.Track_state = 'init'
        if self.Track_state != 'init':
            if len(self.hsv_range) != 0:
                rgb_img, binary, self.circle,corner = self.color.object_follow(rgb_img, self.hsv_range)
                if self.dyn_update == True:
                    write_HSV(self.hsv_text, self.hsv_range)
                    self.Hmin  = rclpy.parameter.Parameter('Hmin', rclpy.Parameter.Type.INTEGER, self.hsv_range[0][0])
                    self.Smin  = rclpy.parameter.Parameter('Smin', rclpy.Parameter.Type.INTEGER, self.hsv_range[0][1])
                    self.Vmin  = rclpy.parameter.Parameter('Vmin', rclpy.Parameter.Type.INTEGER, self.hsv_range[0][2])
                    self.Hmax  = rclpy.parameter.Parameter('Hmax', rclpy.Parameter.Type.INTEGER, self.hsv_range[1][0])
                    self.Smax  = rclpy.parameter.Parameter('Smax', rclpy.Parameter.Type.INTEGER, self.hsv_range[1][1])
                    self.Vmax  = rclpy.parameter.Parameter('Vmax', rclpy.Parameter.Type.INTEGER, self.hsv_range[1][2])
                    all_new_parameters = [self.Hmin, self.Smin, self.Vmin, self.Hmax, self.Smax, self.Vmax]
                    self.set_parameters(all_new_parameters)
                    self.dyn_update = False

        if self.Track_state in ['mouse', 'identify', 'tracking']:
            values = [p.value if isinstance(p, rclpy.Parameter) else int(p) 
                for p in [self.Hmin, self.Smin, self.Vmin, self.Hmax, self.Smax, self.Vmax]]
            self.hsv_range = (
                (values[0], values[1], values[2]), 
                (values[3], values[4], values[5])
            )

        if self.Track_state == 'tracking':
            self.Start_state = True
            if self.circle[2] != 0:
                self.Center_x = self.circle[0]
                self.Center_y = self.circle[1]
                self.Center_r = self.circle[2]


        return rgb_img, binary

    def execute(self, point_x, dist):
        self.get_param()

        linear_x = self.linear_pid.compute(dist, self.minDist)
        angular_z = self.angular_pid.compute(point_x,320)

        # print(f"angular_z= {angular_z} ,linear_x={linear_x}")

        linear_x = max(-0.8, min(linear_x, 0.8))
        angular_z = max(-0.8, min(angular_z, 0.8))

        if abs(dist - self.minDist) < 40: linear_x = 0
        if abs(point_x - 320.0) < 45: angular_z = 0

        # print("linear_x",linear_x)
        
        twist = Twist()

        twist.angular.z = -angular_z * 1.0
        twist.linear.x = linear_x * 1.0
        self.pub_cmdVel.publish(twist)
        self.Robot_Run = True
        
    
    
        

def main(args=None):
    rclpy.init(args=None)
    color_tracker = color_Tracker("ColorTracker")
    print("start it")
    try:
        rclpy.spin(color_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        color_tracker.cancel()