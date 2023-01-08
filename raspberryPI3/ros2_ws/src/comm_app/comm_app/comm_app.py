import rclpy
from rclpy.node import Node
#from interfaces.msg import MessageApp
from interfaces.msg import Package
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
        super().__init__('comm_app')
        self.subscription = self.create_subscription(Package, "/detect_package", self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        #self.publish_package = self.create_publisher(MessageApp, 'comm_app', 10)
        #self.timer = self.create_timer(self.duration, self.listener_callback)
        #self.create_timer(self.duration, self.listener_callback)


    def listener_callback(self, msg: Package):
        #self.get_logger().info('I am in front of the door: "%s"' % msg.data)
        self.get_logger().info(str(msg))
        #m = MessageApp()
        #m.detectDoor = True  #Car in front of the door
        self.lgpio.buzzer = lgpio.gpiochip_open(self.GPIO_HANDLE)

        lgpio.gpio_write(self.buzzer, self.GPIO_PIN, 1)
        self.lgpio.sleep(5)
        lgpio.gpio_write(self.buzzer, self.GPIO_PIN, 0)

        #lgpio.gpio_claim_output(self.buzzer, self.GPIO_PIN, 1, lFlags=0)


def main(args=None):
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
