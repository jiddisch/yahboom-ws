import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import copysign, sqrt, pow, radians
import time
from rclpy.duration import Duration
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import PyKDL
from math import pi

class Calibrateangular(Node):
    def __init__(self, name):
        super().__init__(name)
        # Create publisher
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        # Declare parameters
        self.declare_parameter('rate', 20.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        
        self.declare_parameter('test_angle', 360.0)
        self.test_angle = self.get_parameter('test_angle').get_parameter_value().double_value
        self.test_angle = radians(self.test_angle)
        
        self.declare_parameter('speed', 2.0)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        
        self.declare_parameter('tolerance', 1.6)
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        
        self.declare_parameter('odom_angular_scale_correction', 0.75)
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value
        
        self.declare_parameter('start_test', False)
        
        self.declare_parameter('direction', 1.0)
        self.direction = self.get_parameter('direction').get_parameter_value().double_value
        
        self.declare_parameter('base_frame', 'base_footprint')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        self.declare_parameter('odom_frame', 'odom')
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        
        # Initialize tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.position = Point()
        self.x_start = self.position.x
        self.y_start = self.position.y
        self.first_angle = 0
        
        print("finish init work")
        
        self.reverse = 1
        self.turn_angle = 0
        self.delta_angle = 0
        self.timer = self.create_timer(0.05, self.on_timer)
        
        # Initialize odom_angle with a default value
        self.odom_angle = 0.0
        self.last_angle = self.odom_angle
        self.test_angle *= self.reverse
        self.error = 0
        self.tf_ready = False  # Flag to track if transform is available

    def on_timer(self):
        # Wait for transform to be available
        if not self.tf_ready:
            try:
                now = rclpy.time.Time()
                rot = self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    now,
                    timeout=Duration(seconds=5.0)
                )
                self.tf_ready = True
                self.get_logger().info('transform is ready')
            except TransformException as e:
                self.get_logger().info('transform not ready: {}'.format(str(e)))
                return
        
        self.start_test = self.get_parameter('start_test').get_parameter_value().bool_value
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').get_parameter_value().double_value
        self.test_angle = self.get_parameter('test_angle').get_parameter_value().double_value
        self.test_angle = radians(self.test_angle)  # Convert angle to radians
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        move_cmd = Twist()
        self.test_angle *= self.reverse
        self.error = self.test_angle - self.turn_angle
        
        if self.start_test:
            try:
                self.odom_angle = self.get_odom_angle()
                if self.odom_angle is None:
                    self.get_logger().warn('Failed to get odom angle, skipping update')
                    return
                    
                self.error = self.test_angle - self.turn_angle
                if self.start_test and (abs(self.error) > self.tolerance or self.error == 0):
                    move_cmd.angular.z = copysign(self.speed, self.error)
                    self.cmd_vel.publish(move_cmd)
                    self.delta_angle = self.odom_angular_scale_correction * self.normalize_angle(self.odom_angle - self.first_angle)
                    self.turn_angle += self.delta_angle
                    print("turn_angle: ", self.turn_angle)
                    self.error = self.test_angle - self.turn_angle
                    print("error: ", self.error)
                    self.first_angle = self.odom_angle
                else:
                    self.turn_angle = 0.0
                    self.cmd_vel.publish(Twist())
                    self.start_test = rclpy.parameter.Parameter('start_test', rclpy.Parameter.Type.BOOL, False)
                    all_new_parameters = [self.start_test]
                    self.set_parameters(all_new_parameters)
                    self.reverse = -self.reverse
                    self.first_angle = 0
            except TransformException as e:
                self.get_logger().warn('transform error during test: {}'.format(str(e)))
        else:
            self.cmd_vel.publish(Twist())
            self.start_test = rclpy.parameter.Parameter('start_test', rclpy.Parameter.Type.BOOL, False)
            all_new_parameters = [self.start_test]
            self.set_parameters(all_new_parameters)

    def get_odom_angle(self):
        try:
            now = rclpy.time.Time()
            rot = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                now,
                timeout=Duration(seconds=1.0)
            )
            cacl_rot = PyKDL.Rotation.Quaternion(rot.transform.rotation.x, rot.transform.rotation.y, rot.transform.rotation.z, rot.transform.rotation.w)
            angle_rot = cacl_rot.GetRPY()[2]
            return angle_rot
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('transform not ready: {}'.format(str(e)))
            return None

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

def main():
    rclpy.init()
    class_calibrateangular = Calibrateangular("calibrate_angular")
    try:
        rclpy.spin(class_calibrateangular)
    except KeyboardInterrupt:
        pass
    finally:
        # Fixed: removed parentheses after cmd_vel
        class_calibrateangular.cmd_vel.publish(Twist())
        class_calibrateangular.destroy_node()