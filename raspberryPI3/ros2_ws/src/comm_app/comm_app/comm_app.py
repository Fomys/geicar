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
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.detectDoor = True
        self.buzzer = lgpio.gpiochip_open(self.GPIO_HANDLE)
        lgpio.gpio_write(self.buzzer, self.GPIO_PIN, 1)
        self.lgpio.sleep(5)
        lgpio.gpio_write(self.buzzer, self.GPIO_PIN, 0)

        lgpio.gpio_claim_output(self.buzzer, self.GPIO_PIN, 1, lFlags=0)
        #timer = self.create_timer(self.TIMER, self.check_button)

      '''  
    def check_button(self):

        self.detectDoor = False
        # Set up the pin 6 en pull down
        #lgpio.gpio_claim_alert(buzzer, 6, lgpio.FALLING_EDGE, lFlags=lgpio.SET_BIAS_PULL_DOWN)
        # Add callback for rising edge (0 to 1)
        #c = lgpio.callback(buzzer, 6, lgpio.FALLING_EDGE, self.button_callback)
        #wait for response of the client
        #TODO : Add manual tiemeout when no answer from the client in 10 minutes
        #while lgpio.gpio_read(buzzer, self.GPIO_PIN) == 0:
        #   pass
        if lgpio.gpio_read(self.buzzer, self.GPIO_PIN) == 1:
            self.detectDoor = True
        else:
            self.detectDoor = False
        self.state = lgpio.gpio_read(self.buzzer, self.GPIO_PIN)
        p = Package()
        p.state_pack = self.detectDoor
        self.publish_package.publish(p)
'''

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
