import rclpy
from rclpy.node import Node

from interfaces.msg import Ultrasonic
from interfaces.msg import ActiveSecu
from interfaces.msg import MotorsOrder
from interfaces.msg import StopCar


class ObstacleDetection(Node):
    MINIMAL_DISTANCE = 40
    CAUTION_DISTANCE = 70


    def __init__(self):
        super().__init__('obstacle_detection')

        # Publishers
        # publish informations to StopCar topic
        self.publish_stop_car = self.create_publisher(StopCar, 'stop_car', 10)

        # Subscribers
        self.subscription_us = self.create_subscription(Ultrasonic, 'us_data', self.us_callback, 10)
        self.subscription_activeSecu = self.create_subscription(ActiveSecu, 'active_secu', self.activeSecu_callback, 10)



    def activeSecu_callback(self, msg: ActiveSecu):
        if msg.active:
            MINIMAL_DISTANCE = 40
            CAUTION_DISTANCE = 70
        else:
            MINIMAL_DISTANCE = 0
            CAUTION_DISTANCE = 0



    def us_callback(self, msg: Ultrasonic):

        stop = StopCar()

        if msg.front_left < self.CAUTION_DISTANCE \
                or msg.front_center < self.CAUTION_DISTANCE \
                or msg.front_right < self.CAUTION_DISTANCE:
            stop.slow_front = True
        else :
            stop.slow_front = False


        if msg.rear_left < self.CAUTION_DISTANCE \
                or msg.rear_center < self.CAUTION_DISTANCE \
                or msg.rear_right < self.CAUTION_DISTANCE:
            stop.slow_rear = True
        else :
            stop.slow_rear = False

        if msg.front_left < self.MINIMAL_DISTANCE \
                or msg.front_center < self.MINIMAL_DISTANCE \
                or msg.front_right < self.MINIMAL_DISTANCE:
            stop.stop_car_front = True
        else :
            stop.stop_car_front = False

        if msg.rear_left < self.MINIMAL_DISTANCE \
               or msg.rear_center < self.MINIMAL_DISTANCE \
               or msg.rear_right < self.MINIMAL_DISTANCE:
            stop.stop_car_rear = True
        else :
            stop.stop_car_rear = False

        self.publish_stop_car.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    obstacle_detection = ObstacleDetection()
    rclpy.spin(obstacle_detection)
    obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
