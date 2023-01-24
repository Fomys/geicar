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
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Log
from interfaces.msg import StopCar, SpeedOrder, SpeedInput, MotorsFeedback, Package, AngleOrder, MessageApp, DestCmd, \
    ActiveSecu, DestCmd
import threading

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

positions = {
    "auriol": DestCmd
}


class WebInterfaceNode(Node):
    def start(self):
        rclpy.spin(self)

    def stop(self):
        rclpy.shutdown()

    TIMER = 0.5

    def __init__(self):
        super().__init__('web_interface')
        self.thread = None
        self.rosout_subscription = self.create_subscription(DestCmd, '/set_pos', self.on_set_pos, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.t = TransformStamped()
        timer = self.create_timer(self.TIMER, self.timer_callback)

    def timer_callback(self):
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.t)


# x=-0.45498513576304334
# y=-0.0025589360403252214
# z=0.0
# x=0.0
# y=0.0
# z=-0.003666682128062078
# w=0.9999932776984912

# x=62.290000915527344
# y=52.77000045776367
# z_orien=0.0
# w_orien=1.0

# x=62.74498605129039
# y=52.772559393803995
# z_orien=0.003666682128062078,
# w_orien=6.722301508776951e-06

    def on_set_pos(self, position):
        now = rclpy.time.Time()
        odom_to_base_link = self.tf_buffer.lookup_transform("base_link", "odom", now)
        self.get_logger().info(f'My log message {odom_to_base_link}\n {position}')
        position.x -= odom_to_base_link.transform.translation.x
        position.y -= odom_to_base_link.transform.translation.y
        position.z_orien -= odom_to_base_link.transform.rotation.z
        position.w_orien -= odom_to_base_link.transform.rotation.w
        self.get_logger().info(f'My log message {odom_to_base_link}\n {position}')

        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'map'
        self.t.child_frame_id = 'odom'
        self.t.transform.translation.x = position.x
        self.t.transform.translation.y = position.y
        self.t.transform.translation.z = 0.0
        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = position.z_orien
        self.t.transform.rotation.w = position.w_orien

rclpy.init()
web_interface_node = WebInterfaceNode()
web_interface_node.start()
web_interface_node.stop()
