import rclpy
from rclpy.node import Node

from interfaces.msg import StopCar
from interfaces.msg import SpeedOrder
from interfaces.msg import SpeedInput
from geometry_msgs.msg import Twist


class Security(Node):
    STOP_SPEED = 0.0  # RPM
    CAUTIOUS_SPEED_REAR = -20.0  # RPM
    CAUTIOUS_SPEED_FRONT = 20.0  # RPM
    def __init__(self):
        super().__init__('secu')

        # Variables
        self.stop_ = StopCar()
        self.speed_input = Twist()

        # Publishers
        # Publish to SpeedOrder
        self.publisher_speed_order_ = self.create_publisher(SpeedOrder, 'speed_order', 10)

        # Subscribers
        # subscribe to StopCar topic
        self.subscription_stop_car_ = self.create_subscription(StopCar, 'stop_car', self.stop_car_callback, 10)

        # subscribe to SpeedOrder topic
        self.subscription_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.speed_input_callback, 10)

    def stop_car_callback(self, msg: StopCar):
        # stock StopCar msg value in array
        speed_ = SpeedOrder()
        speed_.speed_order = self.speed_input.linear.x
        self.stop_ = msg

        if self.stop_.stop_car:
            speed_.speed_order = self.STOP_SPEED

        else:
            if self.stop_.slow_rear:
                speed_.speed_order = max(self.speed_input.linear.x, self.CAUTIOUS_SPEED_REAR)  # RPM
            if self.stop_.slow_front:
                speed_.speed_order = min(self.speed_input.linear.x, self.CAUTIOUS_SPEED_FRONT)  # RPM

        self.publisher_speed_order_.publish(speed_)

    def speed_input_callback(self, cmd_vel: Twist):
        # Variables
        self.speed_input.linear.x = cmd_vel.linear.x


def main(args=None):
    rclpy.init(args=args)
    secu = Security()
    rclpy.spin(secu)
    secu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
