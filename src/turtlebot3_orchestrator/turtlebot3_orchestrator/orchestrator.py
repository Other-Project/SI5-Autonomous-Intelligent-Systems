import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lifecycle_msgs.msg import Transition
from typing import Dict

from .lifecycle_manager import LifecycleManager
from .handlers import BaseNodeHandler, PilotNodeHandler

class Orchestrator(Node):
    def __init__(self, managed_nodes_config: Dict[str, dict] = None):
        super().__init__("orchestrator")
        
        self.managed_nodes: Dict[str, LifecycleManager] = {}
        self.node_handlers: Dict[str, BaseNodeHandler] = {}
        
        # Add managed nodes from config
        if managed_nodes_config:
            for node_name, config in managed_nodes_config.items():
                self.add_managed_node(node_name, config)
        
        # Subscription for gestures
        self.create_subscription(
            String, "/gesture/detected", self._gesture_callback, 10
        )
        
        self.get_logger().info(
            f"Orchestrator started. Managing {len(self.managed_nodes)} node(s): "
            f"{', '.join(self.managed_nodes.keys())}"
        )
    
    def add_managed_node(self, node_name: str, config: dict = None):
        """Add a managed node with its specific handler"""
        if config is None:
            config = {'auto_configure': False, 'auto_activate': False}
        
        # Create the lifecycle manager
        manager = LifecycleManager(self, node_name)
        self.managed_nodes[node_name] = manager
        
        # Create the specific handler for the node
        handler = self._create_handler_for_node(node_name)
        if handler:
            self.node_handlers[node_name] = handler
            handler.setup_subscriptions()
            handler.setup_publishers()
        
        self.get_logger().info(f"Added managed node: {node_name}")
        
        # Wait for services and optionally auto-configure/activate
        if manager.wait_for_services(timeout_sec=2.0):
            self.get_logger().info(f"Services available for {node_name}")
            
            if config.get('auto_configure', False):
                manager.set_state(Transition.TRANSITION_CONFIGURE)
                
                if config.get('auto_activate', False):
                    self.create_timer(
                        0.5,
                        lambda: manager.set_state(Transition.TRANSITION_ACTIVATE),
                        once=True
                    )
        else:
            self.get_logger().warn(f"Services not available for {node_name}")
    
    def _create_handler_for_node(self, node_name: str) -> BaseNodeHandler:
        """Factory to create the appropriate handler based on the node name"""
        if node_name == 'turtlebot3_pilot':
            return PilotNodeHandler(self, node_name)
        
        self.get_logger().warn(f"No specific handler for {node_name}, using base")
        return None
    
    def _gesture_callback(self, msg: String):
        """Callback for detected gestures"""
        gesture = msg.data.lower().strip()
        self.get_logger().info(f"Gesture detected: {gesture}")
        
        # Distribute the gesture to all handlers that can process it
        for node_name, handler in self.node_handlers.items():
            if gesture in handler.handle_gestures():
                self._handle_gesture_for_node(node_name, gesture)
    
    def _handle_gesture_for_node(self, node_name: str, gesture: str):
        """Handle a gesture for a specific node"""
        manager = self.managed_nodes.get(node_name)
        handler = self.node_handlers.get(node_name)
        
        if not manager or not handler:
            self.get_logger().error(f"Manager or handler not found for {node_name}")
            return
        
        # Retrieve the current state then process the gesture
        future = manager.get_state()
        future.add_done_callback(
            lambda f: self._process_state_with_action(
                f, node_name, 
                lambda state: handler.on_gesture(gesture, state)
            )
        )
    
    def _process_state_with_action(self, future, node_name: str, action_callback):
        """
        Process the current state with a provided action callback
        """
        try:
            current_state = future.result().current_state
            self.get_logger().info(
                f"[{node_name}] Current state: {current_state.label} ({current_state.id})"
            )
            
            # Determine the transition ID from the action callback if any
            transition_id = action_callback(current_state)
            
            if transition_id is not None:
                self._change_node_state(node_name, transition_id)
                    
        except Exception as e:
            self.get_logger().error(f"[{node_name}] Failed to process state: {e}")
    
    def _change_node_state(self, node_name: str, transition_id: int):
        """Change the state of a node"""
        manager = self.managed_nodes.get(node_name)
        
        if not manager:
            self.get_logger().error(f"Manager not found for {node_name}")
            return None
        
        return manager.set_state(
            transition_id,
            lambda f: self._on_transition_result(f, node_name, transition_id)
        )
    
    def _on_transition_result(self, future, node_name: str, transition_id: int):
        """Callback when a transition is completed"""
        try:
            res = future.result()
            handler = self.node_handlers.get(node_name)
            
            if handler:
                handler.on_transition_complete(res.success, transition_id)
            else:
                if res.success:
                    self.get_logger().info(f"[{node_name}] Transition successful")
                else:
                    self.get_logger().error(f"[{node_name}] Transition failed")
                    
        except Exception as e:
            self.get_logger().error(f"[{node_name}] Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    managed_nodes_config = {
        'turtlebot3_pilot': {
            'auto_configure': True,
            'auto_activate': False
        }
    }
    
    node = Orchestrator(managed_nodes_config)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()