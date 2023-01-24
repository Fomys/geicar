import sys


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


import datetime
import sys

from flask import Flask, render_template, request
from flask_socketio import emit, SocketIO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from rcl_interfaces.msg import Log
from interfaces.msg import StopCar, SpeedOrder, SpeedInput, MotorsFeedback, Package, AngleOrder, MessageApp, DestCmd, \
    ActiveSecu
import threading

positions = {
    "auriol": DestCmd
}


class WebInterfaceNode(Node):
    def start_in_background(self):
        self.thread = threading.Thread(target=rclpy.spin, args=[self]).start()

    def stop(self):
        rclpy.shutdown()

    def __init__(self, socket_io):
        super().__init__('web_interface')
        self.thread = None
        self.logs = []
        self.socket_io = socket_io
        self.last_bip = False
        self.status = {}
        self.rosout_subscription = self.create_subscription(Log, '/rosout', self.on_log, 10)
        self.stop_car_subscription = self.create_subscription(StopCar, '/stop_car', self.on_stop_car, 1)
        self.speed_order_subscription = self.create_subscription(SpeedOrder, "/speed_order", self.on_speed_order, 1)
        self.motor_feedback_subscription = self.create_subscription(MotorsFeedback, "/motors_feedback",
                                                                    self.on_motors_feedback, 1)
        self.speed_order_input_subscription = self.create_subscription(SpeedInput, "/speed_order_input",
                                                                       self.on_speed_order_input, 1)
        self.delivery_subscription = self.create_subscription(Package, "/detect_package", self.on_pacakge_detect, 1)

        self.speed_order_publisher = self.create_publisher(SpeedInput, "/speed_input", 2)
        self.angle_order_publisher = self.create_publisher(AngleOrder, "/angle_order", 2)
        self.bip_publisher = self.create_publisher(MessageApp, "/reach_door", 1)
        self.pos_publisher = self.create_publisher(DestCmd, "/dest_cmd", 1)
        self.secu_publisher = self.create_publisher(ActiveSecu, "/active_secu", 1)

    def goto_auriol(self):
        msg = ActiveSecu()
        msg.active = True
        self.pos_publisher.publish(msg)
        msg = DestCmd()
        msg.x = 66.5
        msg.y = 52.76
        msg.z_orien = 0.0
        msg.w_orien = 1.0
        self.pos_publisher.publish(msg)

    def goto_avant_ascenceur(self):
        msg = ActiveSecu()
        msg.active = True
        self.pos_publisher.publish(msg)
        msg = DestCmd()
        msg.x = 71.9
        msg.y = 52.7
        msg.z_orien = 0.0
        msg.w_orien = 1.0
        self.pos_publisher.publish(msg)

    def goto_devant_ascenceur(self):
        msg = ActiveSecu()
        msg.active = False
        self.pos_publisher.publish(msg)
        msg = DestCmd()
        msg.x = 74.17
        msg.y = 52.76
        msg.z_orien = 0.70
        msg.w_orien = 0.71
        self.pos_publisher.publish(msg)

    def goto_dedans_ascenceur(self):
        msg = ActiveSecu()
        msg.active = False
        self.pos_publisher.publish(msg)
        msg = DestCmd()
        msg.x = 74.45
        msg.y = 55.14
        msg.z_orien = 0.70
        msg.w_orien = 0.71
        self.pos_publisher.publish(msg)

    def goto_sortie_ascenceur(self):
        msg = ActiveSecu()
        msg.active = True
        self.pos_publisher.publish(msg)
        msg = DestCmd()
        msg.x = 74.17
        msg.y = 52.56
        msg.z_orien = 0.70
        msg.w_orien = 0.71
        self.pos_publisher.publish(msg)

    def goto_porte(self):
        pass

    def on_stop_car(self, stop):
        s = {
            "stop_car_rear": {"status": "good", "text": str(stop.stop_car_rear)},
            "stop_car_front": {"status": "good", "text": str(stop.stop_car_front)},
            "slow_rear": {"status": "good", "text": str(stop.slow_rear)},
            "slow_front": {"status": "good", "text": str(stop.slow_front)},
        }
        self.status.update(s)
        self.socket_io.emit("status", s)

    def on_speed_order(self, order):
        s = {
            "speed_order": {
                "status": "good",
                "text": str(order.speed_order)
            }
        }
        self.status.update(s)
        self.socket_io.emit("status", s)

    def on_pacakge_detect(self, status):
        s = {
            "delivery_button": {
                "status": "good",
                "text": str(status.state_pack)
            }
        }
        self.status.update(s)
        self.socket_io.emit("status", s)

    def on_motors_feedback(self, feedback):
        s = {
            "left_rear_odometry": {
                "status": "good",
                "text": str(feedback.left_rear_odometry)
            },
            "right_rear_odometry": {
                "status": "good",
                "text": str(feedback.right_rear_odometry)
            },
            "left_rear_speed": {
                "status": "good",
                "text": str(feedback.left_rear_speed)
            },
            "right_rear_speed": {
                "status": "good",
                "text": str(feedback.right_rear_speed)
            },
            "steering_angle": {
                "status": "good",
                "text": str(feedback.steering_angle)
            },
        }
        self.status.update(s)
        self.socket_io.emit("status", s)

    def on_speed_order_input(self, order):
        s = {
            "speed_order_input": {
                "status": "good",
                "text": str(order.speed_order_input)
            }
        }
        self.status.update(s)
        self.socket_io.emit("status", s)

    def publish_speed(self, speed, angle):
        msg = SpeedInput()
        try:
            msg.speed_order_input = float(speed)
        except ValueError:
            msg.speed_order_input = 0
        self.speed_order_publisher.publish(msg)
        msg = AngleOrder()
        try:
            msg.angle_order = float(angle)
        except ValueError:
            msg.angle_order = 0
        self.angle_order_publisher.publish(msg)

    def bip_on(self):
        msg = MessageApp()
        msg.detect_door = True
        self.bip_publisher.publish(msg)
    def bip_off(self):
        msg = MessageApp()
        msg.detect_door = False
        self.bip_publisher.publish(msg)

    def on_log(self, log):
        if log.level == 10:
            level = "light"
        elif log.level == 20:
            level = "success"
        elif log.level == 30:
            level = "warning"
        elif log.level == 40:
            level = "danger"
        elif log.level == 50:
            level = "danger"
        self.logs.append({
            "stamp": datetime.datetime.fromtimestamp(log.stamp.sec),
            "level": level,
            "msg": log.msg
        })
        self.socket_io.emit("log", {
            "stamp": str(datetime.datetime.fromtimestamp(log.stamp.sec)),
            "level": level,
            "msg": log.msg
        })


rclpy.init()
app = Flask(__name__, template_folder='templates')
app.config.from_pyfile("config.py")
socket_ = SocketIO(app, async_mode=None)
web_interface_node = WebInterfaceNode(socket_io=socket_)


@app.route('/', methods=["GET"])
def index():
    if request.method == 'GET':
        if "auriol" in request.args.keys():
            web_interface_node.goto_auriol()
        elif "avant_ascenceur" in request.args.keys():
            web_interface_node.goto_avant_ascenceur()
        elif "devant_ascenceur" in request.args.keys():
            web_interface_node.goto_devant_ascenceur()
        elif "dedans_ascenceur" in request.args.keys():
            web_interface_node.goto_dedans_ascenceur()
        elif "sortie_ascenceur" in request.args.keys():
            web_interface_node.goto_sortie_ascenceur()
        elif "porte" in request.args.keys():
            web_interface_node.goto_porte()
    return render_template("index.html")

@app.route('/debug', methods=["GET"])
def debug():
    if request.method == 'GET':
        if "bip-on" in request.args.keys():
            web_interface_node.bip_on()
        elif "bip-off" in request.args.keys():
            web_interface_node.bip_on()
        elif "bip-off" in request.args.keys():
            web_interface_node.bip_on()
        elif "map_etage1" in request.args.keys():
            web_interface_node.map_etage1()
        elif "set_initial" in request.args.keys():
            web_interface_node.set_initial()
        elif "set_auriol" in request.args.keys():
            web_interface_node.set_auriol()
        elif "set_avant_ascenceur" in request.args.keys():
            web_interface_node.set_avant_ascenceur()
        elif "set_devant_ascenceur" in request.args.keys():
            web_interface_node.set_devant_ascenceur()
        elif "set_dedans_ascenceur" in request.args.keys():
            web_interface_node.set_dedans_ascenceur()
        elif "map_blank" in request.args.keys():
            web_interface_node.map_blank()
        elif "set_dedans_ascenceur" in request.args.keys():
            web_interface_node.set_dedans_ascenceur_blank()
    return render_template("debug.html")


@app.route('/logs', methods=["GET"])
def logs():
    log_history = web_interface_node.logs
    return render_template("logs.html", log_history=log_history)


@app.route('/status', methods=["GET"])
def status():
    return render_template("status.html", status=web_interface_node.status)


web_interface_node.start_in_background()
app.run(port=5000, host="0.0.0.0")
