import datetime

import rclpy
import signal

from flask import Flask, render_template
from flask_socketio import emit, SocketIO
from rclpy.node import Node

from rcl_interfaces.msg import Log
from interfaces.msg import Status, MotorsFeedback, Ultrasonic
import threading



class WebInterfaceNode(Node):
    def start_in_background(self):
        self.thread = threading.Thread(target=rclpy.spin, args=[self]).start()

    def stop(self):
        rclpy.shutdown()

    def __init__(self, socket_io):
        super().__init__('web_interface')
        self.rosout_subscription = self.create_subscription(Log, '/rosout', self.on_log, 10)
        self.status_subscription = self.create_subscription(Status, '/status', self.on_status, 1)
        self.motor_feedback_subscription = self.create_subscription(MotorsFeedback, '/motors_feedback', self.on_motors_feedback, 1)
        self.ultrasonic_front_subscription = self.create_subscription(Ultrasonic, '/ultrasonic_front', self.on_ultrasonic_front, 1)
        self.ultrasonic_back_subscription = self.create_subscription(Ultrasonic, '/ultrasonic_back', self.on_ultrasonic_back, 1)
        self.thread = None
        self.logs = []
        self.socket_io = socket_io

    def on_ultrasonic_front(self, ultrasonic):
        self.socket_io.emit("status", {
            "ultrasonic_front_left": {"state": "ok", "text": str(ultrasonic.left)},
            "ultrasonic_front_center": {"state": "ok", "text": str(ultrasonic.center)},
            "ultrasonic_front_right": {"state": "ok", "text": str(ultrasonic.right)},
        })

    def on_ultrasonic_back(self, ultrasonic):
        self.socket_io.emit("status", {
            "ultrasonic_back_left": {"state": "ok", "text": str(ultrasonic.left)},
            "ultrasonic_back_center": {"state": "ok", "text": str(ultrasonic.center)},
            "ultrasonic_back_right": {"state": "ok", "text": str(ultrasonic.right)},
        })

    def on_motors_feedback(self, motors_feedback):
        self.socket_io.emit("status", {
            "speed_left": {"state": "ok", "text": str(motors_feedback.left_rear_speed)},
            "speed_right": {"state": "ok", "text": str(motors_feedback.right_rear_speed)},
            "steering_angle": {"state": "ok", "text": str(motors_feedback.steering_angle)},
            "left_rear_odometry": {"state": "ok", "text": str(motors_feedback.left_rear_odometry)},
            "right_rear_odometry": {"state": "ok", "text": str(motors_feedback.right_rear_odometry)},
        })

    def on_status(self, status):
        if status.can_socket_connected == Status.CAN_SOCKET_NOT_RUNNING:
            can_socket_connected = {"state": "fatal", "text": "Not running"}
            stm32f103_status = {"state": "fatal", "text": "CAN not running"}
        elif status.can_socket_connected == Status.CAN_SOCKET_CONNECTED:
            can_socket_connected = {"state": "ok", "text": "Connected"}
            if status.stm32f103_connected:
                stm32f103_status = {"state": "ok", "text": "Alive"}
            else:
                stm32f103_status = {"state": "fatal", "text": "Doesn't respond"}
        elif status.can_socket_connected == Status.CAN_SOCKET_DISCONNECTED:
            stm32f103_status = {"state": "fatal", "text": "CAN disconnected"}
            can_socket_connected = {"state": "error", "text": "Disconnected"}

        self.socket_io.emit("status", {
            "can_status": can_socket_connected,
            "stm32f103_status": stm32f103_status,
        })

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
    return render_template("status.html")


web_interface_node.start_in_background()
app.run(port=5000, host="0.0.0.0")
