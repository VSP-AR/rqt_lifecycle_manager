import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import TransitionEvent, Transition
from lifecycle_msgs.srv import GetState, ChangeState
from ros2lifecycle.api import get_node_names as get_lifecycle_node_names
from ros2node.api import get_absolute_node_name

class LifecycleNodeListNodeManager:
    def __init__(self, node, logger):
        self._node = node
        self._logger = logger
        self.lifecycle_node_states = {}

    def check_if_lifecycle_node(self, node_name):
        absolute_node_name = get_absolute_node_name(node_name)
        node_names = get_lifecycle_node_names(node=self._node, include_hidden_nodes=True)

        if absolute_node_name not in {n.full_name for n in node_names}:
            return False
        
        state = self._get_lifecycle_state(absolute_node_name)

        if state is not None:
            self._logger.info(f'{absolute_node_name} is a lifecycle node.')
            return True
        else:
            self._logger.info(f'{absolute_node_name} is not a lifecycle node.')
            return False
    
    def list_lifecycle_nodes(self):       
        node_names = get_lifecycle_node_names(node=self._node, include_hidden_nodes=True)
        return [n.full_name for n in node_names]
    
    def _get_lifecycle_state(self, node_name):
        client = self._node.create_client(GetState, f'{node_name}/get_state')
        if not client.service_is_ready():
            client.wait_for_service()
        
        request = GetState.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is not None:
            return future.result().current_state
        else:
            self._logger.error(f'Failed to get lifecycle state for {node_name}')
            return None

    def get_lifecycle_state(self, node_name, node_namespace=None):
        name = '/' + node_namespace + '/' + node_name if node_namespace else node_name
        absolute_node_name = get_absolute_node_name(name)
        
        state = self._get_lifecycle_state(absolute_node_name)
        self._node.get_logger().info(f'Node {absolute_node_name} is in state: {state}')
        return state
    
    def set_lifecycle_state(self, node_name, node_namespace=None, transition_label=None):
        name = '/' + node_namespace + '/' + node_name if node_namespace else node_name
        absolute_node_name = get_absolute_node_name(name)
        self._logger.info(f'Node {absolute_node_name} transitioning to {transition_label}')
        
        transition_id = self._get_transition_id_by_label(transition_label)
        if transition_id is None:
            self._logger.error(f'Invalid transition label: {transition_label}')
            return False

        client = self._node.create_client(ChangeState, f'{absolute_node_name}/change_state')
        if not client.service_is_ready():
            client.wait_for_service()
        
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is not None and future.result().success:
            self._logger.info(f'Node {absolute_node_name} transitioned to {transition_label}')
            return True
        else:
            self._logger.error(f'Failed to transition node {absolute_node_name} to {transition_label}')
            return False

    def _get_transition_id_by_label(self, label):
        transition_map = {
            'configure': Transition.TRANSITION_CONFIGURE,
            'activate': Transition.TRANSITION_ACTIVATE,
            'deactivate': Transition.TRANSITION_DEACTIVATE,
            'cleanup': Transition.TRANSITION_CLEANUP,
            'shutdown': Transition.TRANSITION_CONFIGURE, 
            'destroy': Transition.TRANSITION_DESTROY
        }
        return transition_map.get(label.lower())

    def transition_event_callback(self, node_name, msg):
        self.lifecycle_node_states[node_name] = msg
        self._logger.info(f'Node {node_name} transitioned: {msg.transition.label}')
