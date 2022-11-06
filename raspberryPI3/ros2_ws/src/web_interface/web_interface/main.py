import datetime

import rclpy
import signal

from flask import Flask, render_template
from flask_socketio import emit, SocketIO
from rclpy.node import Node

from rcl_interfaces.msg import Log
import threading



class WebInterfaceNode(Node):
    def start_in_background(self):
        self.thread = threading.Thread(target=rclpy.spin, args=[self]).start()

    def stop(self):
        rclpy.shutdown()

    def __init__(self, socket_io):
        super().__init__('web_interface')
        self.subscription = self.create_subscription(Log, '/rosout', self.on_log, 10)
        self.thread = None
        self.logs = []
        self.socket_io = socket_io

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
