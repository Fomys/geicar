import rclpy
from rclpy.node import Node

from interfaces.msg import StopCar
from interfaces.msg import SpeedOrder
from geometry_msgs.msg import Twist
import math


class Security(Node):
    STOP_SPEED = 0.0  # RPM
    CAUTIOUS_SPEED_REAR = -20.0  # RPM
    CAUTIOUS_SPEED_FRONT = 20.0  # RPM
    WHEELBASE = 0.53 #distance entre roues avant et arrières

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

        # subscribe to cmd_vel topic
        self.subscription_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.speed_input_callback, 10)

    def stop_car_callback(self, msg: StopCar):
        # stock StopCar msg value in array

        speed_ = SpeedOrder()

        # speed_input.linear.x is a speed in m/s. We need to transform it as RPM.
        #if speed_input.linear.x is more than 0.65m/s (2.3km/h) then it will be limited
        # because max RPM is 62.
        # circumference of the wheel = 63 cm. 1 RPM is equivalent to the speed (0.63/60) m/s = 0.0105 m/s
        speed_.speed_order = self.speed_input.linear.x/0.0105

        #Vitesse minimale (pour que les roues tournent) en commande  = 20 RPM
        if 2.0 > speed_.speed_order > -2.0:
            speed_.speed_order = 0.0
        elif 20.0 > speed_.speed_order > 2.0:
            speed_.speed_order = 20.0
        elif -20.0 < speed_.speed_order < -2.0:
            speed_.speed_order = -20.0

        elif speed_.speed_order > 62.0:
            speed_.speed_order = 62.0
        elif speed_.speed_order < -62.0:
            speed_.speed_order = -62.0



        #On fait attention aux obstacles et on régule la vitesse en fonction des ultrasons
        #et la direction dans laquelle on avance
        self.stop_ = msg

        if self.stop_.stop_car_front and speed_.speed_order > 0.0:
            #si l'obstacle est devant la voiture et qu'elle avance, on l'empeche d'avancer
            speed_.speed_order = self.STOP_SPEED
        elif self.stop_.stop_car_rear and speed_.speed_order < 0.0:
            #si l'obstacle est derriere la voiture et qu'elle recule, on l'empeche de reculer
            speed_.speed_order = self.STOP_SPEED
        else:
            if self.stop_.slow_rear and speed_.speed_order < 0.0:
                #si l'obstacle est derriere la voiture et qu'elle recule, on la ralentit si elle veut reculer trop rapidement
                speed_.speed_order = max(speed_.speed_order, self.CAUTIOUS_SPEED_REAR)  # RPM
            elif self.stop_.slow_front and speed_.speed_order > 0.0:
                #si l'obstacle est devant la voiture et qu'elle avance, on la ralentit si elle veut avancer trop rapidement
                speed_.speed_order = min(speed_.speed_order, self.CAUTIOUS_SPEED_FRONT)  # RPM



        #On calcule maintenant l'angle des roues qui dépend de la vitesse
        # speed_input.angular.z is the angular speed
        # We know that speed_.angle_order = -1 corresponds to 0.33 rad to the left.
        # We know that speed_.angle_order = 1 is 0.45 rad to the right.
        # speed_input.angular.z gives command between -1 and 1 which corresponds to -pi/2 to pi/2 rad/s.
        # So when we want to turn left : speed_input.angular.z needs to be multiplied by 4.8.
        # When we want to turn right : speed_input.angular.z needs to be multiplied by 3.5.


        if self.speed_input.angular.z == 0.0 or speed_.speed_order == 0.0:
            speed_.angle_order = 0.0 #avoid impossible equation
        else:
            if self.speed_input.angular.z < 0.0:
                speed_.angle_order = -math.atan(self.WHEELBASE * (self.speed_input.angular.z * 3.5)/(speed_.speed_order * 0.0105));
            else:
                speed_.angle_order = -math.atan(self.WHEELBASE * (self.speed_input.angular.z * 4.8) / (speed_.speed_order * 0.0105));



        #On sature la commande, en espérant qu'on ne s'éloigne jamais trop de ces valeurs
        if speed_.angle_order < -1.0:
            speed_.angle_order = -1.0
        elif speed_.angle_order > 1.0:
            speed_.angle_order = 1.0

        self.get_logger().info('speed en sortie de secu =  "%f"' %speed_.speed_order)
        self.publisher_speed_order_.publish(speed_)

    def speed_input_callback(self, cmd_vel: Twist):
        # Variables
        self.speed_input.linear.x = cmd_vel.linear.x
        self.speed_input.angular.z = cmd_vel.angular.z


def main(args=None):
    rclpy.init(args=args)
    secu = Security()
    rclpy.spin(secu)
    secu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
