#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import MessageApp
from interfaces.msg import Package
from interfaces.msg import MotorsFeedback
import lgpio
#import time

#import numpy as np
class CommApp(Node):

    #Class constants
    GPIO_HANDLE = 0
    GPIO_PIN = 23
    #duration = 1   # 1 sec of duration

    #Class variables
    detectDoor = False #Car not in front of the door
    #timer = True

    def __init__(self):
        super().__init__("comm_app")
        self.subscription = self.create_subscription(MessageApp, "/reach_door", self.listener_callback, 10)
        self.buzzer = lgpio.gpiochip_open(self.GPIO_HANDLE)
        lgpio.gpio_claim_output(self.buzzer, self.GPIO_PIN, lFlags=lgpio.SET_BIAS_PULL_DOWN)

        self.status_door = MessageApp()

    def listener_callback(self, msg: MessageApp):
        self.get_logger().info(str(msg))
        self.status_door = msg
        #self.get_logger().info('I am in front of the door: "%s"' % msg.data)
        #m = MessageApp()
        #m.detectDoor = True  #Car in front of the door

        #self.lgpio.buzzer = lgpio.gpiochip_open(self.GPIO_HANDLE)
        if self.status_door.detect_door :
            lgpio.gpio_write(self.buzzer, self.GPIO_PIN, 1)
        else:
            lgpio.gpio_write(self.buzzer, self.GPIO_PIN, 0)

        #lgpio.gpio_write(self.buzzer, self.GPIO_PIN2, 1)
        #self.lgpio.sleep(5)
        #lgpio.gpio_write(self.buzzer, self.GPIO_PIN2, 0)

        #lgpio.gpio_claim_output(self.buzzer, self.GPIO_PIN, 1, lFlags=0)
        m = MessageApp()

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
