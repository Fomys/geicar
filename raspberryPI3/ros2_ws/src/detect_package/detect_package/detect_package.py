import rclpy
from rclpy.node import Node

from interfaces.srv import ButtonPressed
import lgpio
import numpy as np
class DetectPackage(Node):

    PackageIn = False
    time = 0
    done = False

    def __init__(self):
        super().__init__('detect_package')
        self.srv = self.create_service(ButtonPressed, 'add_two_ints', self.check_button)


    def check_button(self, request, response):
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.done = False

        # Set up the pin 6 en pull down
        button = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_alert(button, 6, lgpio.FALLING_EDGE, lFlags=lgpio.SET_BIAS_PULL_DOWN)

        # Add callbac for rising edge (0 to 1)
        c = lgpio.callback(button, 6, lgpio.FALLING_EDGE, self.button_callback)

        while not self.done:
            pass

        return response

    def button_callback(self, a, b, c, timestamp):
        if np.abs(timestamp - self.time) > 1000000000:
            self.PackageIn = not self.PackageIn
            self.get_logger().info("State Package ", self.PackageIn)
            self.done = True


def main():
    rclpy.init()

    detect_package = DetectPackage()
    rclpy.spin(detect_package)

    detect_package.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()