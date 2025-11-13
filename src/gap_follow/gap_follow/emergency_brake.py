import rclpy
from rclpy import time
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import sys
import termios
import tty
import math
import threading

ACCEPTABLE_TTC = 0.5 #s

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/safety_node', 10)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.info_publisher = self.create_publisher(String, 'emergency_brake_info', 10)

        self.speed = 0.0
        self.steering_angle = 0.0
        self.key_pressed = None

        self.ttc = 0.0

        self.speed_odom = 0.0
        self.safety_brake_flag = 0.0

        self.vehicle_width_m = 0.3 # vehicle width in m

        self.ttc_params = {
            'dt': -1.0,                 # The time between laserscan messages
            'dx': -1.0,                 # The forward range delta
            'last_scan_ns': -1.0,       # The time of the last scan meesage in ns
            'last_scan_dist': -1,       # The last forward range
        }

        self.timer = self.create_timer(0.1, self.timer_callback)

    def laser_callback(self, msg):
        self.get_logger().info('Laser callback triggered')
        laser_time = self.get_clock().now().nanoseconds
        laser_dist = self.fwd_range(msg)
        if self.ttc_params['last_scan_ns'] >= 0:
            ttc = math.inf
            try:
                velocity = self.speed_odom
                ttc = laser_dist / velocity

            except Exception as e:
                ttc = -1

            self.safety_brake_flag = ttc > 0 and ttc < ACCEPTABLE_TTC
            self.ttc = ttc

        self.ttc_params['last_scan_dist'] = laser_dist
        self.ttc_params['last_scan_ns'] = laser_time

    def fwd_range(self, msg):
        count_fwd = 0
        ranges = msg.ranges
        fwd_ranges = []
        min_dist = math.inf
        for i in range(len(ranges)):
            angle = self.angle(msg.angle_min, msg.angle_max, i, len(ranges))
            y_dist = ranges[i] * math.sin(angle)
            x_dist = ranges[i] * math.cos(angle)
            if -self.vehicle_width_m / 2 < y_dist and y_dist < self.vehicle_width_m / 2 and x_dist > 0.1:
                count_fwd += 1
                fwd_ranges.append(i)
                if ranges[i] < min_dist:
                    min_dist = ranges[i]

        return min_dist
    
    def angle(self, min, max, index, length):
        angle_step = (max - min) / length 
        return min + angle_step * index

    def odom_callback(self, msg):
        twist = msg.twist
        linear_x = twist.twist.linear.x
        self.speed_odom = linear_x

    def map_angle_to_index(self, angle, angle_min, angle_max, num_readings):
        if angle < angle_min or angle > angle_max:
            return None
        index = int((angle - angle_min) / (angle_max - angle_min) * (num_readings - 1))
        return index

    def timer_callback(self):
        info_msg = String()
        info_msg.data = f'Last Scan Distance: {self.ttc_params["last_scan_dist"]}, Safety Brake: {self.safety_brake_flag}, TTC: {self.ttc}'
        self.info_publisher.publish(info_msg)
        if self.safety_brake_flag or self.ttc_params['last_scan_dist'] < 0.2:
                self.speed = 0.0
                msg = AckermannDriveStamped()
                msg.drive.speed = self.speed
                msg.drive.steering_angle = self.steering_angle
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
