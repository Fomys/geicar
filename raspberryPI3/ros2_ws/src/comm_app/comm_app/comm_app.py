import rclpy
from rclpy.node import Node
from interfaces.msg import MessageApp
import lgpio
import time

import numpy as np
class CommApp(Node):

    #Class constants
    GPIO_HANDLE = 0
    GPIO_PIN = 23

    #Class variables
    detectDoor = False #Car in front of the door

    def __init__(self):
        super().__init__('comm_app')
        self.subscription = self.create_subscription(MessageApp, 'comm_app', self.listener_callback, 10)
        #self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I am in front of the door: "%s"' % msg.data)
        self.detectDoor = True
        self.lgpio.buzzer = lgpio.gpiochip_open(self.GPIO_HANDLE)
        lgpio.gpio_write(self.buzzer, self.GPIO_PIN, 1)
        self.lgpio.sleep(5)
        lgpio.gpio_write(self.buzzer, self.GPIO_PIN, 0)

        lgpio.gpio_claim_output(self.buzzer, self.GPIO_PIN, 1, lFlags=0)
        #timer = self.create_timer(self.TIMER, self.check_button)

def main():
    rclpy.init()

    comm_app = CommApp()
    rclpy.spin(comm_app)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    comm_app.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
