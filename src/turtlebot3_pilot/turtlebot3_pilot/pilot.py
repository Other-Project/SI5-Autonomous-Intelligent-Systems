import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # Indispensable pour transformer des Poses


class Pilot(Node):
    def __init__(self):
        super().__init__("turtlebot3_pilot")

        self.goal_subscriber = self.create_subscription(
            PointStamped, "/robot/goal_point", self.goal_callback, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.current_pose = None
        self.goal_pose = None

        self.get_logger().info("Pilot node started.")

    def odom_callback(self, msg):
        self.current_pose = msg
        self.get_logger().info(f"Current pose updated: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
        if self.goal_pose is not None:
            self.go_to_goal()

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.get_logger().info(f"Received new goal point: {msg.point.x}, {msg.point.y}")
        if self.current_pose is not None:
            self.go_to_goal()

    def go_to_goal(self):
        if self.current_pose is None or self.goal_pose is None:
            self.get_logger().warning("Waiting for current pose and goal pose...")
            return
        
        self.get_logger().info("Transforming goal point to global frame...")

        pose_global = PoseStamped()
        pose_global.header.frame_id = "map"
        pose_global.header.stamp = self.get_clock().now().to_msg()
        pose_global.pose.position.x = self.goal_pose.point.x + self.current_pose.pose.pose.position.x
        pose_global.pose.position.y = self.goal_pose.point.y + self.current_pose.pose.pose.position.y
        pose_global.pose.position.z = 0.0
        pose_global.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal ({pose_global.pose.position.x:.2f}, {pose_global.pose.position.y:.2f}) to Nav2...")

        self._action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_global
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected by Nav2")
            return

        self.get_logger().info("Goal accepted by Nav2")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Goal completed with status: {result.status}")


def main(args=None):
    rclpy.init(args=args)
    node = Pilot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
