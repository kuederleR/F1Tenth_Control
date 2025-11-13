import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('emergency_brake')
        # Declare parameters first
        self.declare_parameters(
            namespace='',
            parameters=[
                ('safety_publish_topic', '/safety_node'),
                ('scan_topic', '/scan'),
                ('acceptable_ttc', 0.5),
            ]
        )
        
        self.publish_topic = self.get_parameter('safety_publish_topic').get_parameter_value().string_value
        self.laser_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        
        self.publisher_ = self.create_publisher(AckermannDriveStamped, self.publish_topic, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.laser_topic,
            self.laser_callback,
            10
        )

        self.info_publisher = self.create_publisher(String, 'emergency_brake_info', 10)

        self.acceptable_ttc = self.get_parameter('acceptable_ttc').get_parameter_value().double_value

        self.get_logger().info('Emergency braking node started.')
        
        self.speed = 0.0
        self.steering_angle = 0.0

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
        laser_time = self.get_clock().now().nanoseconds
        laser_dist = self.fwd_range(msg)
        if self.ttc_params['last_scan_ns'] >= 0:
            ttc = math.inf
            try:
                dt_ns = laser_time - self.ttc_params['last_scan_ns']
                dt = float(dt_ns) / 1000000.0
                dx = laser_dist - self.ttc_params['last_laser_dist']
                velocity = dx / dt
                ttc = laser_dist / velocity

            except:
                ttc = -1

            self.safety_brake_flag = ttc > 0 and ttc < self.acceptable_ttc

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
            if -self.vehicle_width_m / 2 < y_dist and y_dist < self.vehicle_width_m / 2:
                count_fwd += 1
                fwd_ranges.append(i)
                if ranges[i] < min_dist:
                    min_dist = ranges[i]

        return min_dist
    
    def angle(self, min, max, index, length):
        angle_step = (max - min) / length 
        return min + angle_step * index

    def map_angle_to_index(self, angle, angle_min, angle_max, num_readings):
        if angle < angle_min or angle > angle_max:
            return None
        index = int((angle - angle_min) / (angle_max - angle_min) * (num_readings - 1))
        return index

    def timer_callback(self):
        info_msg = String()
        info_msg.data = f'Last Scan Distance: {self.ttc_params["last_scan_dist"]}, Safety Brake: {self.safety_brake_flag}'
        self.info_publisher.publish(info_msg)
        if self.safety_brake_flag:
                
                self.speed = 0.0
                msg = AckermannDriveStamped()
                msg.drive.steering_angle = self.steering_angle
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
