import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.msg import TransitionEvent, Transition
from lifecycle_msgs.srv import ChangeState
from ros2lifecycle.api import get_node_names as get_lifecycle_node_names, call_get_states, call_change_states
from ros2node.api import get_absolute_node_name

class NodeListNode(Node):
    def __init__(self):
        super().__init__('node_list_node')
        self.lifecycle_node_states = {}

    def check_if_lifecycle_node(self, node_name):
        absolute_node_name = get_absolute_node_name(node_name)
        node_names = get_lifecycle_node_names(node=self, include_hidden_nodes=True)

        if absolute_node_name not in {n.full_name for n in node_names}:
            return False
        
        states = call_get_states(node=self, node_names=[absolute_node_name])
        state = states.get(absolute_node_name)

        if state and not isinstance(state, Exception):
            self.get_logger().info(f'{absolute_node_name} is a lifecycle node.')
            return True
        else:
            self.get_logger().info(f'{absolute_node_name} is not a lifecycle node.')
            return False
    
    def get_lifecycle_state(self, node_name, node_namespace=None):
        name = '/' + node_namespace + '/' + node_name if node_namespace else node_name
        absolute_node_name = get_absolute_node_name(name)
        
        states = call_get_states(node=self, node_names=[absolute_node_name])
        self.get_logger().info(f'Node {absolute_node_name} is in states: {states}')
        state = states.get(absolute_node_name)
        self.get_logger().info(f'Node {absolute_node_name} is in state: {state}')
    
    def set_lifecycle_state(self, node_name, node_namespace=None, transition_label=None):
        name = '/' + node_namespace + '/' + node_name if node_namespace else node_name
        absolute_node_name = get_absolute_node_name(name)
        self.get_logger().info(f'Node {absolute_node_name} transitioning to {transition_label}')
        
        transition = Transition()
        transition.label = transition_label
        transition.id = Transition.TRANSITION_CONFIGURE

        result = call_change_states(node=self, transitions={absolute_node_name: transition})
        self.get_logger().info(f'Node {absolute_node_name} transitioned to {transition_label} with result: {result}')
        

    def transition_event_callback(self, node_name, msg):
        self.lifecycle_node_states[node_name] = msg
        self.get_logger().info(f'Node {node_name} transitioned: {msg.transition.label}')

def main(args=None):
    rclpy.init(args=args)
    node = NodeListNode()

    try:
        lifecycle_node_names_and_namespaces = get_lifecycle_node_names(node=node)
        
        for node_info in lifecycle_node_names_and_namespaces:
            node.get_lifecycle_state(node_info.name)
            
            '''
            Todo: get state result
            lifecycle_msgs.msgs.State = states.get('node_name')
            trans = Transition()
            trans.label = 'configure'
            trans.id = Transition.TRANSITION_CONFIGURE 
            
            '''
            # Sleep for 2 seconds
            node.get_logger().info('Sleeping for 2 seconds...')
            rclpy.spin_once(node, timeout_sec=2.0)
            
            node.set_lifecycle_state(node_info.name, transition_label='configure')
            # Sleep for 2 seconds
            node.get_logger().info('Sleeping for 2 seconds...')
            rclpy.spin_once(node, timeout_sec=2.0)
            
            node.set_lifecycle_state(node_info.name, transition_label='activate')
            # Sleep for 10 seconds
            node.get_logger().info('Sleeping for 10 seconds...')
            rclpy.spin_once(node, timeout_sec=10.0)
            
            node.set_lifecycle_state(node_info.name, transition_label='deactivate')
            # Sleep for 2 seconds
            node.get_logger().info('Sleeping for 2 seconds...')
            rclpy.spin_once(node, timeout_sec=2.0)
            
            node.set_lifecycle_state(node_info.name, transition_label='cleanup')
            # Sleep for 2 seconds
            node.get_logger().info('Sleeping for 2 seconds...')
            rclpy.spin_once(node, timeout_sec=2.0)
            
            node.set_lifecycle_state(node_info.name, transition_label='shutdown')
            # Sleep for 2 seconds
            node.get_logger().info('Sleeping for 2 seconds...')
            rclpy.spin_once(node, timeout_sec=2.0)

    except Exception as e:
        node.get_logger().error(f"Exception: {str(e)}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    rclpy.shutdown()
