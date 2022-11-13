import rclpy
from rclpy.node import Node

from interfaces.msg import Ultrasonic
from interfaces.msg import MotorsOrder
from interfaces.msg import StopCar
class ObstacleDetection(Node):

    def __init__(self):
        super().__init__('obstacle_detection')

        #Variables
        self.motors_order_ = MotorsOrder()
        self.stop_ = StopCar()

        #Publishers
        #publish informations to MotorsOrder topic
        self.publisher_can_ = self.create_publisher(MotorsOrder, 'motors_order', 10)
        #publish informations to StopCar topic
        self.stop_car_ = self.create_publisher(StopCar,'stop_car', 10)

        #time_period = 0.05
        #self.timer1 = self.create_timer(time_period, self.timer_callback1)
        #self.i = 0

        #self.distance_ = self.create_publisher(Ultrasonic, 'distance', 10)
        #publish to Brain
        #time_period = 0.05
        #self.timer2  = self.create_timer(time_period, self.timer_callback2)

        #Subscribers
        self.subscription_us_ = self.create_subscription(Ultrasonic, 'us_data', self.us_callback, 10)
        # subscription to Ultrasound Data topic
        self.subscription_us_

        #initiate Ultrasound Distance array to stock value
        self.usDistance = [-1, -1, -1, -1, -1, -1]

    #def timer_callback(self):


    def us_callback(self, msg):
        self.usDistance[0] = msg.front_left
        self.usDistance[1] = msg.front_center
        self.usDistance[2] = msg.front_right
        self.usDistance[3] = msg.rear_left
        self.usDistance[4] = msg.rear_center
        self.usDistance[5] = msg.rear_right

        self.get_logger().info('Distance front left: "%i" cm' % self.usDistance[0])
        self.get_logger().info('Distance front center: "%i" cm' % self.usDistance[1])
        self.get_logger().info('Distance front right: "%i" cm' % self.usDistance[2])
        self.get_logger().info('Distance rear left: "%i" cm' % self.usDistance[3])
        self.get_logger().info('Distance rear center: "%i" cm' % self.usDistance[4])
        self.get_logger().info('Distance rear right: "%i" cm' % self.usDistance[5])

        #self.motors_order_.left_rear_pwm = 100
        #self.motors_order_.right_rear_pwm = 100

                #if self.motors_order_.left_rear_pwm > 70:
                #        self.motors_order_.left_rear_pwm = 70

                #if self.motors_order_.left_rear_pwm < 30:
                #       self.motors_order_.left_rear_pwm = 30

                #if self.motors_order_.right_rear_pwm > 70:
                #        self.motors_order_.right_rear_pwm = 70

                #if self.motors_order_.right_rear_pwm < 30:
                #        self.motors_order_.right_rear_pwm = 30
        self.stop_.slow_front = False
        self.stop_.slow_rear  = False
        self.stop_.stop_car = False

        for i in range(0, 3):
            if self.usDistance[i] < 50:
                self.stop_.slow_front = True

        for i in range(3, 6):
            if self.usDistance[i] < 50:
                self.stop_.slow_rear = True

        for i in range (0, 6):
            if self.usDistance[i] < 20:
                self.stop_.stop_car = True

        self.stop_car_.publish(self.stop_)

        # self.publisher_can_.publish(self.motors_order_)
        self.get_logger().info(f'Publishing to secu node: {self.stop_}')
        #self.i += 1



def main(args=None):
    rclpy.init(args=args)
    obstacle_detection =ObstacleDetection()
    rclpy.spin(obstacle_detection)
    obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
