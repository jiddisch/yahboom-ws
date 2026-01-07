import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from tf2_ros import TransformListener
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32,Bool,Int16,UInt16

from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
from yahboomcar_msgs.msg import FollowerPoint

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.declare_parameter('robot_id', 'robot2')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        if self.robot_id == 'robot2':
            self.target_point = 'point2'
            self.pub_topic = f"/{self.robot_id}/goal_pose"
        elif self.robot_id == 'robot3':
            self.target_point = 'point3'
            self.pub_topic = f"/{self.robot_id}/goal_pose"
        else:
            self.get_logger().error(f"Unsupported robot_id: {self.robot_id}")
            raise ValueError(f"Unsupported robot_id: {self.robot_id}")
        self.pub_robot_pose = self.create_publisher(PoseStamped, self.pub_topic, 10)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        
        # Initialize previous values to track changes
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.prev_w = 0.0
        # Subscribe to current position
        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped,'robot1/amcl_pose',self.odom_callback,10)
        ## Publish new point
        self.pub_back = self.create_publisher(PoseStamped, self.pub_topic, 10)
        # Subscribe to target point
        self.goal_sub = self.create_subscription(PoseStamped,'robot1/goal_pose',self.goal_pose_callback,10)
        # Subscribe to target point
        self.goal_sub2 = self.create_subscription(PoseWithCovarianceStamped,'robot2/amcl_pose',self.odom2_callback,10)
        # Subscribe to queue/dist
        self.queue_subscriber = self.create_subscription(FollowerPoint, '/queue_type', self.queue_callback, 10)

        self.current_pose = None
        self.current_pose2 = None
        self.goal_pose_text = None
        self.back_pose_published = False

        self.queue_flag = False
  
        self.timer = self.create_timer(0.5, self.timer_callback)

    def goal_pose_callback(self, msg: PoseStamped):
        self.goal_pose_text = msg
        self.back_pose_published = False
        self.get_logger().info(f"Saved new target point: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}")


    def odom_callback(self, msg: PoseWithCovarianceStamped):
        if self.goal_pose_text is None:
            self.get_logger().warn("Waiting for robot1/goal_pose data...")
            return
        self.current_pose = msg.pose.pose
        self.current_pose.position.x =  self.current_pose.position.x 
        self.get_logger().info(f"\nLatest target point: x={self.goal_pose_text.pose.position.x:.3f}, y={self.goal_pose_text.pose.position.y:.3f}\nROBOT1 real-time position: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}")
        
    def odom2_callback(self, msg: PoseWithCovarianceStamped):
        if self.goal_pose_text is None:
            self.get_logger().warn("Waiting for robot1/amcl_pose data...")
            return
        self.current_pose2 = msg.pose.pose
        self.current_pose2.position.x =  self.current_pose2.position.x 
        self.get_logger().info(f"\n{self.robot_id} real-time position: x={self.current_pose2.position.x:.3f}, y={self.current_pose2.position.y:.3f}")


    def timer_callback(self):
        if self.current_pose is None:
            self.get_logger().warn("Waiting for robot1/amcl_pose data...")
            return
        
        dx = self.current_pose.position.x - self.goal_pose_text.pose.position.x
        dy = self.current_pose.position.y - self.goal_pose_text.pose.position.y
        dr = math.sqrt(dx**2 + dy**2)
        self.get_logger().info(f"\nStraight-line distance between vehicles: dr={dr}")
        if dr <= 0.50  and  not self.back_pose_published:
            self.get_logger().warn(f"Publishing new target point for {self.robot_id}........................")
            new_pose = self.compute_back_pose(self.goal_pose_text)
            if new_pose is None:
                return
            self.pub_back.publish(new_pose)
            if self.current_pose2 is None:
                return
            if abs(new_pose.pose.position.x - self.current_pose2.position.x) < 0.2 or abs(new_pose.pose.position.y - self.current_pose2.position.y) < 0.2:
                self.get_logger().warn(f"{self.robot_id} reached target point........................")
                self.back_pose_published = True
        else:
            self.get_point()
    def queue_callback(self, msg):
        self.queue = msg.queue
        self.dist = msg.dist
        self.queue_flag = True

    def compute_back_pose(self, pose_msg: PoseStamped):
        """Calculate position x meters behind the target point"""
        if self.queue_flag == False:
            self.get_logger().warn("Waiting for self.dist  self.queue data...")
            return
        q = pose_msg.pose.orientation
        yaw = self.quaternion_to_yaw(q)
        distance  = self.dist

        if self.queue == "column":
            if self.robot_id == 'robot2':
                new_x = pose_msg.pose.position.x - distance * math.cos(yaw)
                new_y = pose_msg.pose.position.y - distance * math.sin(yaw)
            else:
                new_x = pose_msg.pose.position.x - distance * 2 * math.cos(yaw)
                new_y = pose_msg.pose.position.y - distance * 2 * math.sin(yaw)

        elif self.queue == "row":
            # For row formation, calculate side position
            if self.robot_id == 'robot2':
                # robot2: left side (yaw - π/2)
                new_x = pose_msg.pose.position.x + distance * math.cos(yaw - math.pi/2)
                new_y = pose_msg.pose.position.y + distance * math.sin(yaw - math.pi/2)
            else:
                # robot3: right side (yaw + π/2)
                new_x = pose_msg.pose.position.x + distance * math.cos(yaw + math.pi/2)
                new_y = pose_msg.pose.position.y + distance * math.sin(yaw + math.pi/2)
                
        else:
            angle_offset = math.pi / 3
            if self.robot_id == 'robot2':
            # robot2: 后右下角60度位置（yaw + π + angle_offset）
                new_x = pose_msg.pose.position.x + distance *1.5 * math.cos(yaw  +  math.pi + angle_offset)
                new_y = pose_msg.pose.position.y + distance *1.5 * math.sin(yaw  +  math.pi + angle_offset)
            else:
                # robot3: 后左下角60度位置（yaw + math.pi - angle_offset）
                new_x = pose_msg.pose.position.x + distance * math.cos(yaw + math.pi - angle_offset)
                new_y = pose_msg.pose.position.y + distance * math.sin(yaw + math.pi - angle_offset)

        # Create new pose message
        new_pose = PoseStamped()
        new_pose.header = pose_msg.header
        new_pose.pose.position.x = new_x
        new_pose.pose.position.y = new_y
        new_pose.pose.position.z = pose_msg.pose.position.z
        new_pose.pose.orientation = pose_msg.pose.orientation

        # self.get_logger().info(f"\n{self.robot_id} latest target point: x={new_pose}\n")

        return new_pose

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


    def get_point(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', self.target_point, rclpy.time.Time())
            quaternion = [0, 0, transform.transform.rotation.z, transform.transform.rotation.w]
            rotation = R.from_quat(quaternion)
            euler_angles = rotation.as_euler('xyz', degrees=True)
            
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            current_z = transform.transform.rotation.z
            current_w = transform.transform.rotation.w

            if current_z != self.prev_z or current_w != self.prev_w or abs(current_x - self.prev_x)>0.01 or abs(current_y - self.prev_y)>0.01:
                # print("quaternion: ",quaternion)
                # print("transform: ",transform.transform.translation)
                # print("euler_angles: ",euler_angles)
                # print("----------------------")

                self.goal_pose.pose.position.x = transform.transform.translation.x
                self.goal_pose.pose.position.y = transform.transform.translation.y
                self.goal_pose.pose.orientation.z = transform.transform.rotation.z
                self.goal_pose.pose.orientation.w = transform.transform.rotation.w
                self.pub_robot_pose.publish(self.goal_pose)

                self.prev_z = current_z
                self.prev_w = current_w
                self.prev_x = current_x
                self.prev_y = current_y

        except (tf2_ros.TransformException, KeyError) as e:
            self.get_logger().warn(f"Could not transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TfListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()