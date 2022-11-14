import rclpy
from rclpy.node import Node

from interfaces.msg import StopCar
from interfaces.msg import SpeedOrder
from interfaces.msg import SpeedInput


class Security(Node):
    STOP_SPEED = 0.0  # RPM
    CAUTIOUS_SPEED_REAR = -30.0  # RPM
    CAUTIOUS_SPEED_FRONT = 30.0  # RPM
    def __init__(self):
        super().__init__('secu')

        # Variables
        self.stop_ = StopCar()
        self.speed_input = SpeedInput()

        # Publishers
        # Publish to SpeedOrder
        self.publisher_speed_order_ = self.create_publisher(SpeedOrder, 'speed_order', 10)

        # Subscribers
        # subscribe to StopCar topic
        self.subscription_stop_car_ = self.create_subscription(StopCar, 'stop_car', self.stop_car_callback, 10)

        # subscribe to SpeedOrder topic
        self.subscription_speed_input_ = self.create_subscription(SpeedInput, 'speed_input', self.speed_input_callback, 10)

    def stop_car_callback(self, msg: StopCar):
        # stock StopCar msg value in array
        speed_ = SpeedOrder()
        self.stop_ = msg
        #self.get_logger().info('CALLBACK')

        #self.get_logger().info(f'Desired input speed: {speed_input.speed_order_input}')

        self.get_logger().info(f'Stop : {self.stop_.stop_car}')

        if self.stop_.stop_car:
            speed_.speed_order = self.STOP_SPEED
            #self.get_logger().info('STOP!!!!')

        else:
            if self.stop_.slow_rear:
                speed_.speed_order = max(self.speed_input.speed_order_input, self.CAUTIOUS_SPEED_REAR)  # RPM
            if self.stop_.slow_front:
                speed_.speed_order = min(self.speed_input.speed_order_input, self.CAUTIOUS_SPEED_FRONT)  # RPM

        self.publisher_speed_order_.publish(speed_)

    def speed_input_callback(self, speed_input: SpeedInput):
        # Variables
        self.speed_input = speed_input

        # self.get_logger().info(f'Desired input speed: {speed_input.speed_order_input}')
        #
        # self.get_logger().info(f'Stop : {self.stop_.stop_car}')
        #
        # if self.stop_.stop_car:
        #     speed_.speed_order = self.STOP_SPEED
        #     self.get_logger().info('STOP!!!!')
        #
        # else:
        #     if self.stop_.slow_rear:
        #         speed_.speed_order = max(speed_input.speed_order_input, self.CAUTIOUS_SPEED_REAR)  # RPM
        #     if self.stop_.slow_front:
        #         speed_.speed_order = min(speed_input.speed_order_input, self.CAUTIOUS_SPEED_FRONT)  # RPM
        #
        # self.publisher_speed_order_.publish(speed_)


def main(args=None):
    rclpy.init(args=args)
    secu = Security()
    rclpy.spin(secu)
    secu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
