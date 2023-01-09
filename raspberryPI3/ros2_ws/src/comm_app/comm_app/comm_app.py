#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import MessageApp
import lgpio


class CommApp(Node):

    #Class constants
    GPIO_HANDLE = 0
    GPIO_PIN = 23

    #Class variables
    #detectDoor = False #Car not in front of the door

    def __init__(self):
        super().__init__("comm_app")
        self.subscription = self.create_subscription(MessageApp, "/reach_door", self.listener_callback, 10)
        self.buzzer = lgpio.gpiochip_open(self.GPIO_HANDLE)
        lgpio.gpio_claim_output(self.buzzer, self.GPIO_PIN, lFlags=lgpio.SET_BIAS_PULL_DOWN)

        self.status_door = MessageApp()

    def listener_callback(self, msg: MessageApp):
        self.get_logger().info(str(msg))
        self.status_door = msg

        if self.status_door.detect_door :
            lgpio.gpio_write(self.buzzer, self.GPIO_PIN, 1)
        else:
            lgpio.gpio_write(self.buzzer, self.GPIO_PIN, 0)


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
