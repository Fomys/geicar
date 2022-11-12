import rclpy
from rclpy.node import Node

from interfaces.msg import MotorsOrder
from interfaces.msg import StopCar
from interfaces.msg import SpeedOrder
from interfaces.msg import SpeedInput

class Security(Node):

    def __int__(self):
        super().__init__('secu')

        #Variables
        self.stop_ = StopCar()
        self.motors_order_ = MotorsOrder()
        self.speed_ = SpeedOrder()

        #Publishers
        #publish informations to MotorsOrder topic
        self.publisher_can_ = self.create_publisher(MotorsOrder, 'motors_order', 10)
        self.publisher_speed_order_ = self.create_publisher(SpeedOrder, 'speed_order', 10)

        #Subscribers
        #subscribe to StopCar topic
        self.subscription_stop_car_ = self.create_subscription(StopCar, 'stop_car', self.stop_car_callback, 10)
        self.subscription_stop_car_

        #subscribe to SpeedOrder topic
        self.subscription_speed_input_ = self.create_subscription(SpeedInput, 'speed_input', self.speed_input_callback, 10)
        self.subscription_speed_input_

        #initiate StopCar msg array
        self.stop_car_msg = [-1, -1, -1]

    def stop_car_callback(self,msg):
        #stock StopCar msg value in array
        self.stop_car_msg[0] = msg.stop_car
        self.stop_car_msg[1] = msg.slow_rear
        self.stop_car_msg[2] = msg.slow_front

        self.get_logger().info(f'Stop Car: {self.stop_car_msg[0]}')
        self.get_logger().info(f'Slow Rear: {self.stop_car_msg[1]}')
        self.get_logger().info(f'Slow Front: {self.stop_car_msg[2]}')


        # if self.stop_car_msg[0] == true:
        #     self.motors_order_.left_rear_pwm = 50
        #     self.motors_order_.right_rear_pwm = 50
        #     self.publisher_can_.publish(self.motors_order_)
        #
        # if self.motors_order_[1] == true:
        #     self.motors_order_.left_rear_pwm = 30
        #     self.motors_order_.right_rear_pwm = 30
        #     self.publisher_can_.publish(self.motors_order_)
        #
        # if self.motors_order_[2] == true:
        #     self.motors_order_.left_rear_pwm = 70
        #     self.motors_order_.right_rear_pwm = 70
        #     self.publisher_can_.publish(self.motors_order_)

    def speed_input_callback(self, speedinput):
        self.get_logger().info(f'Desired input speed: {self.speedinput}')

        if self.stop_car_msg[0] == True:
            self.speed_ = 0 #RPM
            self.publisher_speed_order_.publish(self.speed_)

        if self.motors_order_[1] == True:
            if self.speedinput <= -30:
                self.speed_ = max(self.speedinput, -30) #RPM
                self.publisher_speed_order_.publish(self.speed_)

        if self.motors_order_[2] == True:
            if self.speedinput >= 30:
                self.speed_ = min(self.speedinput, 30) #RPM
                self.publisher_speed_order_.publish(self.speed_)

        if (self.stop_car_msg[0] == False) and (self.stop_car_msg[1] == False) and  (self.stop_car_msg[2] == False):
            self.speed_ = self.speedinput
            self.publisher_speed_order_.publish(self.speed_)


def main(args=None):
    rclpy.init(args=args)
    secu = Security()
    rclpy.spin(secu)
    secu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
