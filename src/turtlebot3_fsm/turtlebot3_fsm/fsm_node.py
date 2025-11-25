import math
import random
import rclpy
import signal

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlebot3_fsm.diffdrive import wheels_to_twist
import turtlebot3_fsm.lidar_utils as lu

class FSMNode(Node):

    def __init__(self):
        super().__init__('stop_turn_fsm')

        # --- States ---
        self.STOP    = 0
        self.TURN    = 1
        self.FORWARD = 2
        self.state   = self.STOP

        # --- Parameters ---
        self.SPEED_LINEAR        = 0.2
        self.SPEED_ANGULAR_MAX   = 0.1   # max rotation speed
        self.Kp_ANGULAR          = 1.5   # P-controller for turn
        self.ALIGN_TOLERANCE_DEG = 1.0
        self.WHEEL_SEPARATION    = 0.16
        self.DISTANCE_MIN        = 0.4
        self.OFFSET_MAX_DEG      = 3.0   # max random offset
        self.FOV                 = 45.0  # Field of view +/- xxx
        
        # --- Internals ---
        self.last_scan    = None
        self.current_yaw  = 0.0
        self.target_angle = 0.0

        # --- ROS2 subscriptions ---
        self.scan_sub     = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub     = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        
        # --- ROS2 publications ---
        self.vel_pub      = self.create_publisher(Twist,     '/cmd_vel',        10)
        self.best_dir_pub = self.create_publisher(LaserScan, '/best_direction', 10)

        self.timer = self.create_timer(0.05, self.control_cycle)

        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info("FSM node started...")

    # --- Ctrl+C handler ---
    def signal_handler(self, signum, frame):
        self.get_logger().info("Ctrl+C detected, stopping robot...")
        stop_twist = Twist()
        self.vel_pub.publish(stop_twist)
        rclpy.shutdown()

    # --- Callbacks ---
    def scan_callback(self, msg):
        self.last_scan = msg
           
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # --- Main FSM Loop ---
    def control_cycle(self):
        if self.last_scan is None:
            return

        vel_l = 0.0
        vel_r = 0.0

        #==============================
        # STOP
        #==============================
        if self.state == self.STOP:
            self.target_angle = lu.get_angle_of_clear_area_world(self.last_scan, self.current_yaw, self.DISTANCE_MIN, window_deg=self.FOV)
           
            # Add random angular offset
            random_offset = math.radians(random.uniform(-self.OFFSET_MAX_DEG, self.OFFSET_MAX_DEG))
            self.target_angle += random_offset
                     
            self.state = self.TURN
            self.get_logger().info(f"STOP → TURN | Target angle: {math.degrees(self.target_angle):.1f}°")

        #==============================
        # TURN
        #==============================
        elif self.state == self.TURN:
            angle_error = lu.normalize_angle(self.target_angle - self.current_yaw)
            angle_error_deg = math.degrees(angle_error)

            if abs(angle_error_deg) >= self.ALIGN_TOLERANCE_DEG:
                vel = self.Kp_ANGULAR * angle_error  # proportional control
                # limit angular speed
                vel = max(min(vel, self.SPEED_ANGULAR_MAX), -self.SPEED_ANGULAR_MAX)
                vel_r = vel
                vel_l = -vel
                self.get_logger().info(f"TURN towards {math.degrees(self.target_angle):.1f}° | error {angle_error_deg:.1f}°")
            else:
                vel_r = 0.0
                vel_l = 0.0
                self.state = self.FORWARD
                self.get_logger().info(f"Aligned with target {math.degrees(self.target_angle):.1f}° → FORWARD")

        #==============================
        # FORWARD
        #==============================
        elif self.state == self.FORWARD:
            # check obstacle in FOV
            dist, indices = lu.get_min_range_at_angle(self.last_scan, 0.0, self.FOV)
            if dist is None:
                dist = lu.get_max_range(self.last_scan)  # assume free if no data

            self.get_logger().info(f"Obstacle ahead @({dist:.2f} m)")
                
            if dist < self.DISTANCE_MIN:
                vel_l = 0.0
                vel_r = 0.0
                self.state = self.STOP
                self.get_logger().info(f"Obstacle ahead ({dist:.2f} m) → STOP")
            else:
                vel_l = self.SPEED_LINEAR
                vel_r = self.SPEED_LINEAR
                self.get_logger().info("FORWARD")

        twist = wheels_to_twist(vel_l, vel_r, self.WHEEL_SEPARATION)
        self.vel_pub.publish(twist)
        self.publish_clear_area()

    # --- Debug: publish the best direction ---
    def publish_clear_area(self):
        if self.target_angle is None:
            return

        d, indices = lu.get_avg_range_at_angle(self.last_scan, self.target_angle, 1.0)
        lu.publish_debug_scan(self.last_scan, indices, self.best_dir_pub)

def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
