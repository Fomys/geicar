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


import math
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    q = [0,0,0,0]
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
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
        self.t.header.frame_id = 'map'
        self.t.child_frame_id = 'odom'
        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 0.0
        self.t.transform.translation.x = 0.0
        self.t.transform.translation.y = 0.0
        self.t.transform.translation.z = 0.0
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
        try:
            map_to_base_link = self.tf_buffer.lookup_transform("base_link", "map", now)
            map_to_odom = self.tf_buffer.lookup_transform("odom", "map", now)
            dx = map_to_base_link.transform.translation.x - map_to_odom.transform.translation.x
            dy = map_to_base_link.transform.translation.y - map_to_odom.transform.translation.z
            dz = map_to_base_link.transform.translation.z - map_to_odom.transform.translation.y



            rx,ry,rz = euler_from_quaternion(map_to_base_link.transform.rotation.x, map_to_base_link.transform.rotation.y, map_to_base_link.transform.rotation.z, map_to_base_link.rotation.w)
            rx2,ry2,rz2 = euler_from_quaternion(map_to_odom.transform.rotation.x, map_to_odom.transform.rotation.y, map_to_odom.transform.rotation.z, map_to_odom.rotation.w)
            rpx,rpy,rpz = euler_from_quaternion(0, 0, position.z_orien, position.w_orien)
            dz = rpz-(rz-rz2)
            self.t.transform.rotation.x, self.t.transform.rotation.y, self.t.transform.rotation.z, self.t.transform.rotation.w = quaternion_from_euler(0,0,dz)

            position.x -= dx
            position.y -= dy

            self.t.header.stamp = self.get_clock().now().to_msg()
            self.t.header.frame_id = 'map'
            self.t.child_frame_id = 'odom'
            self.t.transform.translation.x = position.x
            self.t.transform.translation.y = position.y
            self.t.transform.translation.z = 0.0
            self.t.transform.rotation.x = 0.0
            self.t.transform.rotation.y = 0.0
        except:
            self.t.header.frame_id = 'map'
            self.t.child_frame_id = 'odom'
            self.t.transform.translation.x = position.x
            self.t.transform.translation.y = position.y
            self.t.transform.translation.z = 0.0
            self.t.transform.rotation.x = 0.0
            self.t.transform.rotation.y = 0.0
            self.t.transform.rotation.z = position.z
            self.t.transform.rotation.w = position.x

rclpy.init()
web_interface_node = WebInterfaceNode()
web_interface_node.start()
web_interface_node.stop()
