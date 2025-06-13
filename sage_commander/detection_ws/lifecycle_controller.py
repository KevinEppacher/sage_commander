from concurrent.futures import ThreadPoolExecutor
from functools import partial
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

        self.declare_parameter('target_nodes', ['/target_node'])
        self.declare_parameter('timeout_sec', 90.0)
        self.target_nodes = self.get_parameter('target_nodes').get_parameter_value().string_array_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        self.get_logger().info(f'{GREEN}Managing nodes: {self.target_nodes}{RESET}')
        self.managers = []

        for name in self.target_nodes:
            mgr = LifecycleNodeManager(self, name, self.timeout_sec)
            self.managers.append(mgr)

        self.run_all()

    def run_all(self):
        with ThreadPoolExecutor(max_workers=len(self.managers)) as executor:
            futures = [executor.submit(mgr.run) for mgr in self.managers]
            for f in futures:
                f.result()


class LifecycleNodeManager:
    def __init__(self, node, target_node, timeout_sec):
        self.node = node
        self.target_node = target_node
        self.timeout_sec = timeout_sec
        self.cli_change = node.create_client(ChangeState, f'{target_node}/change_state')
        self.cli_get = node.create_client(GetState, f'{target_node}/get_state')

    def run(self):
        log = self.node.get_logger()
        log.info(f'{YELLOW}Waiting for {self.target_node}...{RESET}')
        start_time = time.time()
        while time.time() - start_time < self.timeout_sec:
            if self.cli_change.service_is_ready() and self.cli_get.service_is_ready():
                log.info(f'{GREEN}{self.target_node} services ready{RESET}')
                break
            time.sleep(0.5)
        else:
            log.error(f'{RED}{self.target_node} services timeout{RESET}')
            return

        self.transition(Transition.TRANSITION_CONFIGURE)
        self.transition(Transition.TRANSITION_ACTIVATE)

    def transition(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.cli_change.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout_sec)
        success = future.result() and future.result().success
        if success:
            label = self.get_state()
            self.node.get_logger().info(f'{GREEN}{self.target_node} → {label}{RESET}')
        else:
            self.node.get_logger().error(f'{RED}{self.target_node} failed transition{RESET}')

    def get_state(self):
        req = GetState.Request()
        future = self.cli_get.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout_sec)
        return future.result().current_state.label if future.result() else 'unknown'


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
