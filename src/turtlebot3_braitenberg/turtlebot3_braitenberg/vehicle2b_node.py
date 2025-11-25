import math
import rclpy
import signal

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from braitenberg.diffdrive import wheels_to_twist
from braitenberg.transfer_functions import tf_linear_excitation, tf_linear_inhibition
from braitenberg.vehicle2a import Vehicle
from braitenberg.lidar_utils import get_max_range, get_min_range, get_avg_range_at_angle, publish_debug_scan

# ==== CONSTANTS ====
ANGLE_DEG_LEFT 		= 45.0
ANGLE_WINDOW_LEFT       = 45.0

ANGLE_DEG_RIGHT		= 45.0
ANGLE_WINDOW_RIGHT      = 45.0

WHEEL_SEPARATION 	= 0.16   # meters
MAX_WHEEL_SPEED 	= 0.5    # m/s
SCAN_TOPIC 		= "/scan"
DEBUG_SCAN_TOPIC        = "/braitenberg_sensors"
CMD_VEL_TOPIC 		= "/cmd_vel"
# ===================

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

class BraitenbergNode(Node):
    def __init__(self):
        super().__init__('braitenberg_node')

        # Fixed transfer function and vehicle type
        self.transfer_fn = tf_linear_excitation
        self.vehicle = Vehicle(self.transfer_fn, MAX_WHEEL_SPEED)

        self.pub_cmd = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.sub_scan = self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_cb, qos_profile)
        self.pub_debug_scan = self.create_publisher(LaserScan, DEBUG_SCAN_TOPIC, 10)

        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info("Braitenberg node started...")

    def signal_handler(self, signum, frame):
        self.get_logger().info("Ctrl+C detected, stopping robot...")
        stop_twist = Twist()
        try:
            self.pub_cmd.publish(stop_twist)
        except Exception:
            pass
        rclpy.shutdown()
            
        
    def scan_cb(self, msg: LaserScan):
        left_d, left_indices = get_avg_range_at_angle(msg, ANGLE_DEG_LEFT, ANGLE_WINDOW_LEFT)
        right_d, right_indices = get_avg_range_at_angle(msg, 360.0-ANGLE_DEG_RIGHT, ANGLE_WINDOW_RIGHT)
        max_range = get_max_range(msg)
        min_range = get_min_range(msg)
        
        v_l, v_r = self.vehicle.compute_wheel_speeds(left_d, right_d, 0.000576)
        twist = wheels_to_twist(v_l, v_r, WHEEL_SEPARATION)
        self.pub_cmd.publish(twist)

        publish_debug_scan(msg, [left_indices, right_indices], self.pub_debug_scan)

def main(args=None):
    rclpy.init(args=args)
    node = BraitenbergNode()
    rclpy.spin(node)
    node.destroy_node()
    

