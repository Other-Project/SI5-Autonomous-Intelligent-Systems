from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from lifecycle_msgs.msg import State, Transition
import math
from typing import Optional

from .base_node_handler import BaseNodeHandler

class PilotNodeHandler(BaseNodeHandler):
    """Handler for the TurtleBot3 Pilot node"""
    
    def __init__(self, orchestrator_node, node_name: str = 'turtlebot3_pilot'):
        super().__init__(orchestrator_node, node_name)
        
        self.distance_threshold = 0.2
        self.current_pose = None
        self.last_sent_goal = None
        
        self.goal_publisher = None
        self.goal_subscriber = None
        self.odom_subscriber = None
    
    def setup_subscriptions(self):
        """Configure subscriptions for the pilot"""
        self.goal_subscriber = self.orchestrator.create_subscription(
            PointStamped, "/robot/goal_point", self._handle_goal_callback, 10
        )
        
        self.odom_subscriber = self.orchestrator.create_subscription(
            Odometry, "/odom", self._odom_callback, 10
        )
        
        self.logger.info(f"[{self.node_name}] Subscriptions configured")
    
    def setup_publishers(self):
        """Configure publishers for the pilot"""
        self.goal_publisher = self.orchestrator.create_publisher(
            PoseStamped, "/pilot/goal_point", 10
        )
        
        self.logger.info(f"[{self.node_name}] Publishers configured")
    
    def on_gesture(self, gesture: str, current_state: State) -> Optional[int]:
        """Handle gestures for the pilot"""
        state_id = current_state.id
        
        if gesture == "stop":
            if state_id == State.PRIMARY_STATE_ACTIVE:
                self.logger.info(f"[{self.node_name}] STOP -> Deactivating")
                self.last_sent_goal = None
                return Transition.TRANSITION_DEACTIVATE
            else:
                self.logger.info(f"[{self.node_name}] STOP ignored (not active)")
                return None
                
        elif gesture == "fist":
            if state_id == State.PRIMARY_STATE_INACTIVE:
                self.logger.info(f"[{self.node_name}] FIST -> Activating")
                return Transition.TRANSITION_ACTIVATE
            elif state_id == State.PRIMARY_STATE_UNCONFIGURED:
                self.logger.info(f"[{self.node_name}] FIST -> Configuring first")
                return Transition.TRANSITION_CONFIGURE
            else:
                self.logger.info(f"[{self.node_name}] FIST ignored (already active)")
                return None
        
        return None
    
    def on_transition_complete(self, success: bool, transition_id: int):
        """Handle completed transitions"""
        super().on_transition_complete(success, transition_id)
        
        # If we just configured after a FIST, activate next
        if success and transition_id == Transition.TRANSITION_CONFIGURE:
            self.logger.info(f"[{self.node_name}] Configuration done, now activating...")
            manager = self.orchestrator.managed_nodes.get(self.node_name)
            if manager:
                manager.set_state(
                    Transition.TRANSITION_ACTIVATE,
                    lambda f: self.on_transition_complete(
                        f.result().success, 
                        Transition.TRANSITION_ACTIVATE
                    )
                )
    
    def handle_gestures(self):
        """Return the list of gestures this handler responds to"""
        return ["stop", "fist"]

    def _odom_callback(self, msg: Odometry):
        """Callback for odometry"""
        self.current_pose = msg
    
    def _handle_goal_callback(self, goal_point: PointStamped):
        """Callback for new goals"""
        manager = self.orchestrator.managed_nodes.get(self.node_name)
        if not manager:
            self.logger.warn(f"[{self.node_name}] Manager not found")
            return
        
        future = manager.get_state()
        future.add_done_callback(
            lambda f: self._process_goal(f, goal_point)
        )
    
    def _process_goal(self, future, goal_point: PointStamped):
        """Process a goal if the node is active"""
        if future.result().current_state.id != State.PRIMARY_STATE_ACTIVE:
            return
        
        if self.current_pose is None or goal_point is None:
            self.logger.debug("Waiting for current pose and/or goal pose...")
            return
        
        pose_global = self._convert_point_to_pose(goal_point)
        
        # Check distance from the last goal
        if self.last_sent_goal is not None:
            dx = pose_global.pose.position.x - self.last_sent_goal.pose.position.x
            dy = pose_global.pose.position.y - self.last_sent_goal.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < self.distance_threshold:
                self.logger.debug(
                    "New goal is too close to the last sent goal. Not sending."
                )
                return
            
            self.logger.info(
                f"Distance check: new=({pose_global.pose.position.x:.3f}, "
                f"{pose_global.pose.position.y:.3f}), "
                f"last=({self.last_sent_goal.pose.position.x:.3f}, "
                f"{self.last_sent_goal.pose.position.y:.3f}), "
                f"dist={dist:.3f}m, threshold={self.distance_threshold}m"
            )
        
        self.last_sent_goal = pose_global
        self.goal_publisher.publish(pose_global)
    
    def _convert_point_to_pose(self, point_msg: PointStamped) -> PoseStamped:
        """Convert a PointStamped to a PoseStamped"""
        pose_msg = PoseStamped()
        pose_msg.header = point_msg.header
        pose_msg.pose.position = point_msg.point
        pose_msg.pose.orientation.w = 1.0
        return pose_msg