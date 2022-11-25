import sys

from interfaces.srv import ButtonPressed
import rclpy
from rclpy.node import Node


class Brain(Node):

    state_pack = False

    def __init__(self):
        super().__init__('brain')
        self.cli = self.create_client(ButtonPressed, 'detect_package')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ButtonPressed.Request()

    def send_request(self):
        self.req.state_package = self.state_pack
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    brain = Brain()
    response = brain.send_request()
    brain.get_logger().info('State Package {0}'.format(response.package_update))

    brain.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()