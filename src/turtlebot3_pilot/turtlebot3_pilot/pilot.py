import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import tf2_ros
import math


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
        self.goal_point = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1, self.go_to_goal)

        self.get_logger().info("Pilot node started.")

        self.last_sent_goal = None
        self.distance_threshold = 0.2

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg
        self.get_logger().debug(
            f"Current pose updated: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}"
        )

    def goal_callback(self, msg: PointStamped):
        self.goal_point = msg
        self.get_logger().debug(
            f"Received new goal point: {msg.point.x}, {msg.point.y}"
        )

    def go_to_goal(self):
        if self.current_pose is None or self.goal_point is None:
            self.get_logger().debug("Waiting for current pose and/or goal pose...")
            return

        pose_global = self.convert_point_to_pose(self.goal_point)

        if self.last_sent_goal is not None:
            dx = pose_global.pose.position.x - self.last_sent_goal.pose.position.x
            dy = pose_global.pose.position.y - self.last_sent_goal.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < self.distance_threshold:
                self.get_logger().debug(
                    "New goal is too close to the last sent goal. Not sending a new goal."
                )
                return

        self.get_logger().info(
            f"Sending goal ({pose_global.pose.position.x:.2f}, {pose_global.pose.position.y:.2f}) to Nav2..."
        )

        self.last_sent_goal = pose_global
        self._action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_global
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            return

        self.get_logger().debug("Goal accepted by Nav2")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Goal completed with status: {result.status}")

    def convert_point_to_pose(self, point_msg: PointStamped):
        pos_point_msg = PoseStamped()
        pos_point_msg.header = point_msg.header
        pos_point_msg.pose.position = point_msg.point
        pos_point_msg.pose.position.z = 0.0
        pos_point_msg.pose.orientation.x = 0.0
        pos_point_msg.pose.orientation.y = 0.0
        pos_point_msg.pose.orientation.z = 0.0
        pos_point_msg.pose.orientation.w = 1.0
        return pos_point_msg


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
