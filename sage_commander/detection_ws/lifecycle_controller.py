import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import time

BLUE = "\033[94m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
RESET = "\033[0m"

class LifecycleController(Node):
    def __init__(self):
        super().__init__('lifecycle_controller')

        self.declare_parameter('target_node', '/target_node')
        self.declare_parameter('timeout_sec', 90.0)
        self.target_node = self.get_parameter('target_node').get_parameter_value().string_value   
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value
        self.get_logger().info(f'{GREEN}Using target_node: {self.target_node}{RESET}')
        self.get_logger().info(f'{GREEN}Using timeout_sec: {self.timeout_sec}{RESET}')

        self.cli_change = self.create_client(ChangeState, f'{self.target_node}/change_state')
        self.cli_get = self.create_client(GetState, f'{self.target_node}/get_state')

        self.get_logger().info(f'{YELLOW}Waiting for services {self.cli_change.srv_name} and {self.cli_get.srv_name}...{RESET}')
        time.sleep(3)  # Allow time for the clients to be created

        self.run()

    def wait_for_services(self, timeout_sec=90.0, interval_sec=0.5):
        """Waits until both lifecycle services are available or times out."""
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.cli_change.service_is_ready() and self.cli_get.service_is_ready():
                self.get_logger().info(f'{GREEN}Lifecycle services are now available.{RESET}')
                return True
            time.sleep(interval_sec)
        self.get_logger().error(f'{RED}Lifecycle services not available after timeout.{RESET}')
        return False


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
        if not self.wait_for_services():
            return

        self.get_logger().info(f'Current state: {self.get_state()}')

        self.get_logger().info(f'{YELLOW}Configuring...{RESET}')
        if self.change_state(Transition.TRANSITION_CONFIGURE):
            self.get_logger().info(f'{GREEN}State after configure: {self.get_state()}{RESET}')
        else:
            self.get_logger().error(f'{RED}Failed to configure{RESET}')
            return

        self.get_logger().info(f'{YELLOW}Activating...{RESET}')
        if self.change_state(Transition.TRANSITION_ACTIVATE):
            self.get_logger().info(f'{GREEN}State after activate: {self.get_state()}{RESET}')
        else:
            self.get_logger().error(f'{RED}Failed to activate{RESET}')
            return

        self.get_logger().info(f'{GREEN}Lifecycle transitions done{RESET}')

def main(args=None):
    rclpy.init(args=args)
    controller = LifecycleController()
    controller.destroy_node()
    rclpy.shutdown()
