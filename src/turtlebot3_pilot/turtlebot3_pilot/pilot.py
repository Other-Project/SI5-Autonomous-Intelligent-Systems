import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn


class Pilot(LifecycleNode):
    def __init__(self):
        super().__init__("turtlebot3_pilot")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_point = None
        self.timer = None
        self._goal_handle = None
        self._last_sent_goal = None
        self._cancel_future = None 
        self.get_logger().info("Pilot node started.")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring Pilot...")
        self.goal_subscriber = self.create_subscription(
            PoseStamped, "/pilot/goal_point", self.goal_callback, 10
        )
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating Pilot")
        self._goal_handle = None
        self._cancel_future = None

        self.goal_point = self._last_sent_goal
        self._last_sent_goal = None

        super().on_activate(state)
        self.timer = self.create_timer(1, self.go_to_goal)
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating Pilot")
        
        if self.timer:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        
        if self._goal_handle is not None:
            self.get_logger().info("Cancelling current Nav2 goal...")
            self._cancel_future = self._goal_handle.cancel_goal_async()
        
            self._goal_handle = None
            self._cancel_future = None
        else:
            self.get_logger().info("No active goal to cancel")
        
        self.goal_point = None
                
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Pilot node cleaned up.")

        self.destroy_subscription(self.goal_subscriber)
        return TransitionCallbackReturn.SUCCESS

    def goal_callback(self, msg: PoseStamped):
        if self._state_machine.current_state[1] != 'active':
            return
            
        self.goal_point = msg
        self.get_logger().info(f"New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

    def go_to_goal(self):
        if self._state_machine.current_state[1] != 'active':
            return

        if self.goal_point is None:
            self.get_logger().debug("Waiting for goal...")
            return

        if self._last_sent_goal is not None and self.goal_point.pose == self._last_sent_goal.pose:
            self.get_logger().debug("Goal is the same as the last sent goal. Not sending again.")
            return

        self.get_logger().info("Sending goal to Nav2...")
        
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 action server not available!")
            return
        
        self._last_sent_goal = self.goal_point
             
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_point
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            return

        self.get_logger().info("Goal accepted by Nav2")
        self._goal_handle = goal_handle
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self._goal_handle = None
        
        if result.status == 4:
            self.get_logger().info("Goal reached successfully!")
        elif result.status == 6:
            self.get_logger().info("Goal was cancelled")
        else:
            self.get_logger().info(f"Goal completed with status: {result.status}")


def main(args=None):
    rclpy.init(args=args)
    node = Pilot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()