import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

class LifecycleController(Node):
    def __init__(self):
        super().__init__('seem_lifecycle_controller')

        self.target_node = '/seem_lifecycle_node'
        self.timeout_sec = 90.0

        self.cli_change = self.create_client(ChangeState, f'{self.target_node}/change_state')
        self.cli_get = self.create_client(GetState, f'{self.target_node}/get_state')

        self.run()

    def wait_for_service(self, client):
        if not client.wait_for_service(timeout_sec=self.timeout_sec):
            self.get_logger().error(f'Service {client.srv_name} not available within timeout')
            return False
        return True

    def change_state(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.cli_change.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        return future.result() and future.result().success

    def get_state(self):
        req = GetState.Request()
        future = self.cli_get.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        return future.result().current_state.label if future.result() else 'unknown'

    def run(self):
        if not self.wait_for_service(self.cli_change) or not self.wait_for_service(self.cli_get):
            return

        self.get_logger().info(f'Current state: {self.get_state()}')

        self.get_logger().info('Configuring...')
        if self.change_state(Transition.TRANSITION_CONFIGURE):
            self.get_logger().info(f'State after configure: {self.get_state()}')
        else:
            self.get_logger().error('Failed to configure')
            return

        self.get_logger().info('Activating...')
        if self.change_state(Transition.TRANSITION_ACTIVATE):
            self.get_logger().info(f'State after activate: {self.get_state()}')
        else:
            self.get_logger().error('Failed to activate')
            return

        self.get_logger().info('Lifecycle transitions done')

def main(args=None):
    rclpy.init(args=args)
    controller = LifecycleController()
    controller.destroy_node()
    rclpy.shutdown()
