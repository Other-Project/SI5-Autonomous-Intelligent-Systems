import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ClosestPointPublisher(Node):
    def __init__(self):
        super().__init__('closest_point_laserscan_publisher')

        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.closest_scan_pub = self.create_publisher(LaserScan, '/closest_point', 10)

        # Has message been already initialized?
        self.closest_scan_msg = None

        self.get_logger().info('ClosestPointPublisher node started.')

    def scan_callback(self, msg: LaserScan):
        if self.closest_scan_msg is None:
            # Initialize the message once (optimization...)
            self.closest_scan_msg = LaserScan()
            self.closest_scan_msg.header.frame_id = msg.header.frame_id
            self.closest_scan_msg.angle_min = msg.angle_min
            self.closest_scan_msg.angle_max = msg.angle_max
            self.closest_scan_msg.angle_increment = msg.angle_increment
            self.closest_scan_msg.time_increment = msg.time_increment
            self.closest_scan_msg.scan_time = msg.scan_time
            self.closest_scan_msg.range_min = msg.range_min
            self.closest_scan_msg.range_max = msg.range_max

            # Pre-allocate lists...
            n = len(msg.ranges)
            self.closest_scan_msg.ranges = [float('inf')] * n
            if msg.intensities:
                self.closest_scan_msg.intensities = [0.0] * n
            else:
                self.closest_scan_msg.intensities = []

        # Closest point initialization
        min_distance = float('inf')
        closest_index = -1

        for i, distance in enumerate(msg.ranges):
            if math.isfinite(distance) and distance < min_distance:
                min_distance = distance
                closest_index = i

        self.get_logger().info(f'{closest_index:.1f}')

        if closest_index == -1:
            return

        # Update the pre-allocated message
        n = len(self.closest_scan_msg.ranges)
        for i in range(n):
            self.closest_scan_msg.ranges[i] = float('inf')
            if self.closest_scan_msg.intensities:
                self.closest_scan_msg.intensities[i] = 0.0

        self.closest_scan_msg.ranges[closest_index] = min_distance
        
        if msg.intensities:
            self.closest_scan_msg.intensities[closest_index] = msg.intensities[closest_index]

        # Update the timestamp
        self.closest_scan_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish it!
        self.closest_scan_pub.publish(self.closest_scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClosestPointPublisher()
    try:
        rclpy.spin(node)
    finally:    
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


