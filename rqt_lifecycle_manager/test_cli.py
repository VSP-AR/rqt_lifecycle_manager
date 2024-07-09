import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import TransitionEvent
from ros2node.api import get_node_names

class NodeListNode(Node):
    def __init__(self):
        super().__init__('node_list_node')
        self.lifecyle_node_states = {}

    def check_if_lifecycle_node(self, node_name):
        client = self.create_client(GetState, f'/{node_name}/get_state')
        ready = client.wait_for_service(timeout_sec=3.0)
        if not ready:
            self.get_logger().info(f'{node_name} is not a lifecycle node.')
            return False

        request = GetState.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.result() is not None:
            self.get_logger().info(f'{node_name} is a lifecycle node.')
            return True
        else:
            self.get_logger().info(f'Failed to get state for {node_name}.')
            return False

    def create_lifecycle_subscriber(self, node_name):
        transition_event_topic = f'/{node_name}/transition_event'
        self.create_subscription(
            TransitionEvent,
            transition_event_topic,
            lambda msg: self.transition_event_callback(node_name, msg),
            10
        )

    def transition_event_callback(self, node_name, msg):
        self.lifecyle_node_states[node_name] = msg
        self.get_logger().info(f'Node {node_name} transitioned: {msg.transition.id}')

def main(args=None):
    rclpy.init(args=args)
    node = NodeListNode()

    try:
        node_names_and_namespaces = get_node_names(node=node)
        print("List of ROS 2 nodes:")
        for node_info in node_names_and_namespaces:
            # Filter out the current node itself
            if node_info.name == node.get_name():
                continue

            print(f"Node name: {node_info.name}, Namespace: {node_info.namespace}")
            if node.check_if_lifecycle_node(node_info.name):
                node.create_lifecycle_subscriber(node_info.name)

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        node.get_logger().error(f"Exception: {str(e)}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
