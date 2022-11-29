import rclpy
from rclpy.node import Node

from interfaces.msg import Package
import lgpio
import numpy as np
class DetectPackage(Node):

    #Class constants
    GPIO_HANDLE = 0
    GPIO_PIN = 6
    TIMER = 1

    #Class varaibles
    PackageIn = False #to remove after probably
    state = False
    timer = 1
    done = False

    def __init__(self):
        super().__init__('detect_package')

        #Create service to wait for the input of the package.
        #self.srv = self.create_service(ButtonPressed, 'detect_package', self.check_button)

        timer = create_timer(TIMER, self.check_button())

        self.publish_package = self.create_publisher(Package, 'detect_package', 10)
        self.publish_package.publish(self.PackageIn)



    def check_button(self):
        self.get_logger().info('Incoming request')
        #self.done = False
        self.PackageIn = False

        # Set up the pin 6 en pull down
        button = lgpio.gpiochip_open(self.GPIO_HANDLE)

        #lgpio.gpio_claim_alert(button, 6, lgpio.FALLING_EDGE, lFlags=lgpio.SET_BIAS_PULL_DOWN)
        lgpio.gpio_claim_input(button, self.GPIO_PIN, lFlags=lgpio.SET_BIAS_PULL_DOWN)

        # Add callback for rising edge (0 to 1)
        #c = lgpio.callback(button, 6, lgpio.FALLING_EDGE, self.button_callback)

        #wait for response of the client
        #TODO : Add manual tiemeout when no answer from the client in 10 minutes
        #while lgpio.gpio_read(button, self.GPIO_PIN) == 0:
        #   pass
        self.state = lgpio.gpio_read(button, self.GPIO_PIN)

        if lgpio.gpio_read(button, self.GPIO_PIN and self.state == False) == 1:
            self.PackageIn = not self.PackageIn

        self.publish_package.publish(self.PackageIn)
        self.get_logger().info("Action done")


    #Callback function for the GPIO interrupt
    # def button_callback(self, a, b, c, timestamp):
    #     if np.abs(timestamp - self.timer) > 1000000000:
    #         self.PackageIn = not self.PackageIn
    #         self.get_logger().info("State Package ", self.PackageIn)
    #         self.done = True
    #         self.timer = timestamp


def main():
    rclpy.init()

    detect_package = DetectPackage()
    rclpy.spin(detect_package)

    detect_package.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()