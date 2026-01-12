from abc import ABC, abstractmethod
from rclpy.node import Node
from lifecycle_msgs.msg import State
from typing import List, Optional


class BaseNodeHandler(ABC):
    """Base class to handle node-specific logic"""
    
    def __init__(self, orchestrator_node: Node, node_name: str):
        self.orchestrator = orchestrator_node
        self.node_name = node_name
        self.logger = orchestrator_node.get_logger()

    @abstractmethod
    def on_gesture(self, gesture: str, current_state: State) -> Optional[int]:
        """
        Handle a gesture for this node.
        Returns the transition ID to perform, or None if no action.
        """
        pass
    
    @abstractmethod
    def setup_subscriptions(self):
        """Configure node-specific subscriptions"""
        pass
    
    @abstractmethod
    def setup_publishers(self):
        """Configure node-specific publishers"""
        pass
    
    def handle_gestures(self) -> List[str]:
        """Return the list of gestures this handler responds to"""
        return []

    def on_transition_complete(self, success: bool, transition_id: int):
        """Callback called after a transition (can be overridden)"""
        if success:
            self.logger.info(f"[{self.node_name}] Transition {transition_id} successful")
        else:
            self.logger.error(f"[{self.node_name}] Transition {transition_id} failed")