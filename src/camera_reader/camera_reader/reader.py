import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class CameraReader(Node):
    def __init__(self):
        super().__init__('camera_reader')

        self.subscriber = self.create_subscription(LaserScan, '/camera/nn/image_raw', self.callback, 10)
        #self.publisher = self.create_publisher(LaserScan, '/closest_point', 10)

        self.get_logger().info('CameraReader node started.')

    def callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraReader()
    try:
        rclpy.spin(node)
    finally:    
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


