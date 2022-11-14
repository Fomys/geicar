import rclpy
from rclpy.node import Node

from interfaces.msg import Ultrasonic
from interfaces.msg import MotorsOrder
from interfaces.msg import StopCar


class ObstacleDetection(Node):
    MINIMAL_DISTANCE = 30
    CAUTION_DISTANCE = 50

    stop = StopCar()
    stop.slow_front = False
    stop.slow_rear = False
    stop.stop_car = False

    def __init__(self):
        super().__init__('obstacle_detection')

        # Publishers
        # publish informations to StopCar topic
        self.publish_stop_car = self.create_publisher(StopCar, 'stop_car', 10)

        # Subscribers
        self.subscription_us = self.create_subscription(Ultrasonic, 'us_data', self.us_callback, 10)

    def us_callback(self, msg: Ultrasonic):

        if msg.front_left < self.CAUTION_DISTANCE \
                or msg.front_center < self.CAUTION_DISTANCE \
                or msg.front_right < self.CAUTION_DISTANCE:
            self.stop.slow_front = True

        if msg.rear_left < self.CAUTION_DISTANCE \
                or msg.rear_center < self.CAUTION_DISTANCE \
                or msg.rear_right < self.CAUTION_DISTANCE:
            self.stop.slow_rear = True

        if msg.front_left < self.MINIMAL_DISTANCE \
                or msg.front_center < self.MINIMAL_DISTANCE \
                or msg.front_right < self.MINIMAL_DISTANCE \
                or msg.rear_left < self.MINIMAL_DISTANCE \
                or msg.rear_center < self.MINIMAL_DISTANCE \
                or msg.rear_right < self.MINIMAL_DISTANCE:
            self.stop.stop_car = True
            self.get_logger().info(f'Stop : {self.stop.stop_car}')

        self.publish_stop_car.publish(self.stop)


def main(args=None):
    rclpy.init(args=args)
    obstacle_detection = ObstacleDetection()
    rclpy.spin(obstacle_detection)
    obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
