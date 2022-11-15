import datetime

import rclpy
import signal

from flask import Flask, render_template, request
from flask_socketio import emit, SocketIO
from rclpy.node import Node

from rcl_interfaces.msg import Log
from interfaces.msg import StopCar, SpeedOrder, SpeedInput, MotorsFeedback
import threading


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
        self.status = {}
        self.rosout_subscription = self.create_subscription(Log, '/rosout', self.on_log, 10)
        self.stop_car_subscription = self.create_subscription(StopCar, '/stop_car', self.on_stop_car, 1)
        self.speed_order_subscription = self.create_subscription(SpeedOrder, "/speed_order", self.on_speed_order, 1)
        self.motor_feedback_subscription = self.create_subscription(MotorsFeedback, "/motors_feedback", self.on_motors_feedback, 1)
        self.speed_order_input_subscription = self.create_subscription(SpeedInput, "/speed_order_input",
                                                                       self.on_speed_order_input, 1)
        self.speed_order_publisher = self.create_publisher(SpeedInput, "/speed_input", 1)

    def on_stop_car(self, stop):
        s = {
            "stop_car": {"status": "good", "text": str(stop.stop_car)},
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
                "text": str(feedback.steering_angl)
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

    def publish_speed(self, speed):
        msg = SpeedInput()
        try:
            msg.speed_order_input = float(speed)
        except ValueError:
            msg.speed_order_input = 0
        self.speed_order_publisher.publish(msg)

    def on_log(self, log):
        if log.level == 10:
            level = "debug"
        elif log.level == 20:
            level = "info"
        elif log.level == 30:
            level = "warn"
        elif log.level == 40:
            level = "error"
        elif log.level == 50:
            level = "fatal"
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
    return render_template("index.html")


@app.route('/logs', methods=["GET"])
def logs():
    log_history = web_interface_node.logs
    return render_template("logs.html", log_history=log_history)


@app.route('/status', methods=["GET"])
def status():
    return render_template("status.html", status=web_interface_node.status)


@app.route('/set_speed', methods=["GET", "POST"])
def set_speed():
    if request.method == 'POST':
        speed = request.form.get('speed')
        web_interface_node.publish_speed(speed)
    return render_template("speed.html", status=web_interface_node.status)


web_interface_node.start_in_background()
app.run(port=5000, host="0.0.0.0")
