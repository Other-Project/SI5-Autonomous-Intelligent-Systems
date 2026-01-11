from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from typing import Optional, Callable


class LifecycleManager:
    def __init__(self, node: Node, target_node_name: str):
        self.node = node
        self.target_node_name = target_node_name
        self.logger = node.get_logger()
        
        self._client_get_state = node.create_client(
            GetState, f"/{target_node_name}/get_state"
        )
        self._client_change_state = node.create_client(
            ChangeState, f"/{target_node_name}/change_state"
        )
        
    def wait_for_services(self, timeout_sec: float = 5.0) -> bool:
        """Wait for the lifecycle services of the managed node to be available"""
        return (
            self._client_get_state.wait_for_service(timeout_sec=timeout_sec) and
            self._client_change_state.wait_for_service(timeout_sec=timeout_sec)
        )
    
    def get_state(self, callback: Optional[Callable] = None):
        """Get the current state of the managed node"""
        future = self._client_get_state.call_async(GetState.Request())
        if callback:
            future.add_done_callback(callback)
        return future
    
    def set_state(self, transition_id: int, callback: Optional[Callable] = None):
        """Set the state of the managed node using a transition ID"""
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self._client_change_state.call_async(req)
        if callback:
            future.add_done_callback(callback)
        return future