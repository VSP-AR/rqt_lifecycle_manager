import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.msg import TransitionEvent, Transition
from lifecycle_msgs.srv import ChangeState
from ros2lifecycle.api import get_node_names as get_lifecycle_node_names, call_get_states, call_change_states
from ros2node.api import get_absolute_node_name

class LifecycleNodeListNodeManager:
    def __init__(self, node, logger):
        self._node = node
        self._logger = logger
        self.lifecycle_node_states = {}

    def check_if_lifecycle_node(self, node_name):
        absolute_node_name = get_absolute_node_name(node_name)
        node_names = get_lifecycle_node_names(node=self, include_hidden_nodes=True)

        if absolute_node_name not in {n.full_name for n in node_names}:
            return False
        
        states = call_get_states(node=self, node_names=[absolute_node_name])
        state = states.get(absolute_node_name)

        if state and not isinstance(state, Exception):
            self._logger.info(f'{absolute_node_name} is a lifecycle node.')
            return True
        else:
            self._logger.info(f'{absolute_node_name} is not a lifecycle node.')
            return False
    
    def list_lifecycle_nodes(self):       
        node_names = get_lifecycle_node_names(node=self._node, include_hidden_nodes=True)
        return [n.full_name for n in node_names]
    
    def get_lifecycle_state(self, node_name, node_namespace=None):
        name = '/' + node_namespace + '/' + node_name if node_namespace else node_name
        absolute_node_name = get_absolute_node_name(name)
        
        states = call_get_states(node=self._node, node_names=[absolute_node_name])
        self._node.get_logger().info(f'Node {absolute_node_name} is in states: {states}')
        state = states.get(absolute_node_name)
        self._node.get_logger().info(f'Node {absolute_node_name} is in state: {state}')
    
    def set_lifecycle_state(self, node_name, node_namespace=None, transition_label=None):
        name = '/' + node_namespace + '/' + node_name if node_namespace else node_name
        absolute_node_name = get_absolute_node_name(name)
        self._logger.info(f'Node {absolute_node_name} transitioning to {transition_label}')
        
        transition = Transition()
        transition.label = transition_label
        transition.id = Transition.TRANSITION_CONFIGURE

        result = call_change_states(node=self._node, transitions={absolute_node_name: transition})
        self._logger.info(f'Node {absolute_node_name} transitioned to {transition_label} with result: {result}')
        

    def transition_event_callback(self, node_name, msg):
        self.lifecycle_node_states[node_name] = msg
        self._logger.info(f'Node {node_name} transitioned: {msg.transition.label}')