from numpy import rad2deg
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import cos, sin, pi, log, exp, atan2
from dataclasses import dataclass

DEBUG = False

"""
Basic logic: 
find gaps in lidar big enoug for car.
score these gaps, mostly favoring gaps on the left so the car doesn't leave the track.
Get the angle of the best gap and use a PID loop with the error as the angle of the gap.
Use a bunch of hard-coded numbers to make it work.
    like ofsetting the error by 0.15rad so the car stays on the right side.
    or look for gaps within 2.5m

    There's some extra things in this code because I want to add some sort of speed
    control based on various factors like gap selection uncertainty and gap parameters.
    but I haven't made that work yet.
"""

class PIDControlNode(Node):
    def __init__(self):
        super().__init__('pid_control')

        # Declare parameters - values will come from YAML if provided, otherwise use defaults
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gap_follow_base_speed', 7.0),
                ('gap_follow_min_speed', 0.4),
                ('gap_follow_max_steering_angle', 0.4),
                ('gap_follow_kp', 0.8),
                ('gap_follow_ki', 0.005),
                ('gap_follow_kd', 0.02),
                ('ego_car_width', 0.3),
                ('gap_follow_selection_multiplier', 1.5),
                ('gap_follow_lookahead_distance', 3.0),
                ('gap_follow_gap_detection_distance', 2.5),
                ('gap_follow_aim_offset', -0.15),
            ]
        )


        self.base_speed = self.get_parameter('gap_follow_base_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter('gap_follow_min_speed').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('gap_follow_max_steering_angle').get_parameter_value().double_value

        self.kp = self.get_parameter('gap_follow_kp').get_parameter_value().double_value
        self.ki = self.get_parameter('gap_follow_ki').get_parameter_value().double_value
        self.kd = self.get_parameter('gap_follow_kd').get_parameter_value().double_value

        self.car_width = self.get_parameter('ego_car_width').get_parameter_value().double_value

        self.selection_multiplier = self.get_parameter('gap_follow_selection_multiplier').get_parameter_value().double_value
        
        self.car_width = self.get_parameter('ego_car_width').get_parameter_value().double_value
        
        self.lookahead_distance = self.get_parameter('gap_follow_lookahead_distance').get_parameter_value().double_value
        self.gap_detection_distance = self.get_parameter('gap_follow_gap_detection_distance').get_parameter_value().double_value

        self.aim_offset = self.get_parameter('gap_follow_aim_offset').get_parameter_value().double_value
        
        
       
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        self.car = Car(
            velocity=0.0,
            acceleration=0.0,
            width=self.car_width,
            collision_state=False,
            collision_distance=0.0,
        )

        self.get_logger().info('PID Control Node Started')
        
        # PID state
        self.error_integral = 0.0
        self.last_error = 0.0
        self.last_time = None

        self.current_gap_wr = 0.4
        
        # Gap detection parameters
        self.min_gap_width = self.car.width * self.selection_multiplier  # Minimum gap width

        self.gap_uncertainty = 0.0
        
        # Current control values
        self.current_steering_angle = 0.0
        self.current_speed = self.base_speed
        self.target_gap_center = 0.0  # Angle to center of chosen gap
        self.gap_dist = 0.0
        
        self.odom_params = {
            'lt': 0,
            'lv': 0.0,
            'init': True
        }

        self.current_gap = None
        
        # Debug counters
        self.debug_counter = 0
        
        # Control timer
        self.drive_timer = self.create_timer(0.05, self.drive_callback)  # 20Hz control

    def drive_callback(self):
        """Main control loop - publishes drive commands"""
        msg = AckermannDriveStamped()
        
        # Calculate steering using PID control
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.last_time is not None:
            dt = current_time - self.last_time
            
            # Error is the angle to center of target gap
            # Should calculate aim offset here.
            
            gap_rotation = self.current_gap['edge_rotation'] - pi/2
            # gap_rotation = rad2deg(gap_rotation)
            lun = 1.0
            try:
                lun = log(self.gap_uncertainty)
            except: pass

            self.get_logger().info(str(lun))

            # error = self.target_gap_center + self.aim_offset # Aim a little right
            error = self.target_gap_center + self.aim_offset
            # error = self.target_gap_center - lun * gap_rotation

            # PID calculation
            self.error_integral += error * dt
            # Limit integral windup
            self.error_integral = max(-0.5, min(0.5, self.error_integral))
            
            error_derivative = (error - self.last_error) / dt if dt > 0 else 0.0
            
            steering_angle = (self.kp * error + 
                            self.ki * self.error_integral + 
                            self.kd * error_derivative)
            
            # Limit steering angle
            steering_angle = max(-self.max_steering_angle, 
                               min(self.max_steering_angle, steering_angle))
            
            self.current_steering_angle = steering_angle
            self.last_error = error
            
        self.last_time = current_time
        
        # Speed control based on steering angle and proximity to walls
        self.calculate_speed()
        
        # Publish command
        msg.drive.speed = self.current_speed
        msg.drive.steering_angle = self.current_steering_angle
        self.publisher_.publish(msg)
        
        # Debug output every 20 cycles (1 second)
        self.debug_counter += 1
        if self.debug_counter % 20 == 0 and DEBUG:
            self.get_logger().info(
                f'Gap center: {self.target_gap_center:.3f}, '
                f'Steering: {self.current_steering_angle:.3f}, '
                f'Speed: {self.current_speed:.2f}'
            )

    def calculate_speed(self):
        """Calculate speed based on steering angle and proximity to obstacles"""
        # Base speed reduction based on steering angle
        # steering_factor = 1.0 - 0.4 * abs(self.current_steering_angle) / self.max_steering_angle
        # steering_factor = 1.0 / (2.5 * self.current_gap_wr)

        uncertainty = self.gap_uncertainty

        gap_angle = 0.0
        if not self.current_gap == None:
            gap_angle = abs(self.current_gap['center_angle'])
        exp_fn = 2.0 - (1.0 / exp(gap_angle))
        steering_factor = 1.0 / exp_fn
        # Speed reduction based on proximity to walls
        if hasattr(self, 'min_distance_ahead'):
            distance_factor = min(1.0, max(0.3, self.min_distance_ahead / 1.5))
        else:
            distance_factor = 1.0
        
        # Combine factors
        speed_factor = min(steering_factor, distance_factor) 
        # - log(self.gap_uncertainty)
        uncertainty_reduction = 0.0
        try: uncertainty_reduction = 0.00 * log(uncertainty) * self.base_speed
        except: pass
        self.current_speed = self.base_speed * speed_factor

    def laser_callback(self, msg):
        """Process lidar data for gap detection"""
        # Find all valid gaps
        gaps = self.find_gaps(msg.ranges, msg.angle_min, msg.angle_increment)
        
        # Choose the best gap (prioritizing left)
        chosen_gap, self.gap_uncertainty = self.choose_best_gap(gaps)
        self.current_gap = chosen_gap
        
        self.current_gap_wr = chosen_gap['width_ratio']
        if chosen_gap:
            self.target_gap_center = chosen_gap['center_angle']
        else:
            # No valid gaps - emergency straight or slight left
            self.target_gap_center = 0.1  # Slight left bias
        
        # Calculate minimum distance ahead for speed control
        forward_ranges = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) < pi/4 and r > 0.1:
                forward_ranges.append(r)
                
        if forward_ranges:
            self.min_distance_ahead = min(forward_ranges)
        else:
            self.min_distance_ahead = float('inf')

    def find_gaps(self, ranges, angle_min, angle_increment):
        """Find all navigable gaps in the lidar data"""
        gaps = []
        
        # Convert lidar data to obstacle points
        obstacles = []
        for i, r in enumerate(ranges):
            if 0.1 < r < self.gap_detection_distance:
                angle = angle_min + i * angle_increment
                x = r * cos(angle)
                y = r * sin(angle)
                obstacles.append((x, y, angle, r))
        
        if len(obstacles) < 2:
            # No obstacles or very few - consider everything a gap
            return [{
                'left_angle': -pi/3,
                'right_angle': pi/3,
                'center_angle': 0.0,
                'width': 2*pi/3,
                'angular_width': 2*pi/3,
                'width_ratio': 0.0,          # default to 0 for consistency
                'edge_rotation': 0.0,        # straight ahead by convention
                'score': 1.0
            }]
        
        # Sort obstacles by angle
        obstacles.sort(key=lambda obs: obs[2])
        
        # Find gaps between consecutive obstacles
        for i in range(len(obstacles)):
            current_obs = obstacles[i]
            next_obs = obstacles[(i + 1) % len(obstacles)]
            
            # Handle wrap-around case
            if i == len(obstacles) - 1:
                # Gap from last obstacle to first obstacle (wrapping around)
                if current_obs[2] < next_obs[2]:
                    continue  # Skip if not actually wrapping
                left_angle = current_obs[2]
                right_angle = next_obs[2] + 2*pi
            else:
                left_angle = current_obs[2]
                right_angle = next_obs[2]
            
            # Calculate gap properties
            gap_angular_width = abs(right_angle - left_angle)
            gap_center_angle = (left_angle + right_angle) / 2.0
            
            # Normalize center angle to [-pi, pi]
            while gap_center_angle > pi:
                gap_center_angle -= 2*pi
            while gap_center_angle < -pi:
                gap_center_angle += 2*pi
            
            # Calculate physical gap width at a reference distance
            reference_distance = min(current_obs[3], next_obs[3]) if i < len(obstacles) - 1 else current_obs[3]
            gap_physical_width = reference_distance * gap_angular_width

            # Compute rotation (angle from left edge to right edge in Cartesian space)
            left_x, left_y = current_obs[0], current_obs[1]
            right_x, right_y = next_obs[0], next_obs[1]
            edge_rotation = atan2(right_y - left_y, right_x - left_x)
        
            width_ratio = gap_angular_width / gap_physical_width if gap_physical_width > 0 else 0.0

            # Only consider gaps that are wide enough and not too narrow
            if gap_physical_width > self.min_gap_width and gap_angular_width > 0.1:
                gaps.append({
                    'left_angle': left_angle,
                    'right_angle': right_angle if i < len(obstacles) - 1 else right_angle - 2*pi,
                    'center_angle': gap_center_angle,
                    'width': gap_physical_width,
                    'angular_width': gap_angular_width,
                    'width_ratio': width_ratio,
                    'edge_rotation': edge_rotation,   # NEW: rotation of edge line (radians, [-pi, pi])
                    'distance': reference_distance,
                    'score': 0.0
                })
        
        return gaps

    def choose_best_gap(self, gaps):
        """Choose the best gap prioritizing left turns"""
        if not gaps:
            return None
        
        # Score each gap
        for gap in gaps:
            gap['score'] = self.score_gap(gap)
        
        # Debug output
        if self.debug_counter % 40 == 0 and gaps and DEBUG:
            self.get_logger().info(f'Found {len(gaps)} gaps:')
            sorted_gaps = sorted(gaps, key=lambda g: g['score'], reverse=True)
            for i, gap in enumerate(sorted_gaps[:3]):
                self.get_logger().info(
                    f'  Gap {i}: center={gap["center_angle"]:.3f}, '
                    f'width={gap["width"]:.3f}, score={gap["score"]:.3f}'
                )
        
        
        # Return the highest scoring gap and uncertainty
        uncertainty = 0.0
        best_gap = max(gaps, key=lambda g: g['score'])
        if len(gaps) > 1:
            min_score_gap = 1000 # Arbitrary large number
            for i, gap in enumerate(gaps):
                score_gap = best_gap['score'] - gap['score']
                
                min_score_gap = score_gap if score_gap < min_score_gap and not gap == best_gap else min_score_gap
            if min_score_gap == 0.0:
                uncertainty = 0.0
            else:
                uncertainty = 1.0 / min_score_gap
        

        
        return best_gap if best_gap['score'] > 0.1 else None, uncertainty

    def score_gap(self, gap):
        """Score a gap based on preference for left turns and gap quality"""
        center_angle = gap['center_angle']
        width = gap['width']
        distance = 0.0
        try:
            distance = gap['distance']
        except:
            pass
        
        # 1. Width score - prefer wider gaps
        width_score = min(1.0, width / (self.car.width * 3.0))
        
        # 2. Direction preference - HEAVILY favor left side gaps
        if center_angle > 0:  # Left side (positive angles)
            if 0 < center_angle <= pi/4:  # 0-45 degrees - perfect range
                direction_score = 1.0
            elif pi/4 < center_angle <= pi/2:  # 45-90 degrees - still good
                direction_score = 0.8
            elif pi/2 < center_angle <= 3*pi/4:  # 90-135 degrees - acceptable
                direction_score = 0.6
            else:  # > 135 degrees - too far back
                direction_score = 0.2
        else:  # Right side (negative angles)
            if -pi/6 <= center_angle < 0:  # Slight right turn acceptable
                direction_score = 0.4
            else:  # Further right - discouraged
                direction_score = 0.1
        
        # 3. Forward preference - discourage backwards gaps
        if abs(center_angle) <= pi/2:  # Forward hemisphere
            forward_score = 1.0
        else:  # Backward hemisphere
            forward_score = 0.2
        
        # 4. Distance score - prefer gaps that aren't too close or too far
        if 0.5 <= distance <= 2.0:
            distance_score = 1.0
        elif distance < 0.5:
            distance_score = 0.5  # Too close
        else:
            distance_score = max(0.3, 2.0 / distance)  # Too far
        
        # 5. Angular width bonus - prefer gaps with good angular clearance
        angular_bonus = min(1.0, gap['angular_width'] / (pi/3))  # Bonus for wide angular gaps
        
        # Combine all scores with weights
        total_score = (
            0.3 * width_score +
            1.1 * direction_score +  # Highest weight on direction
            0.15 * forward_score +
            0.1 * distance_score +
            0.05 * angular_bonus
        )
        
        return total_score

@dataclass
class Car:
    velocity: float
    acceleration: float
    width: float
    collision_state: bool
    collision_distance: float

def main(args=None):
    rclpy.init(args=args)
    node = PIDControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
