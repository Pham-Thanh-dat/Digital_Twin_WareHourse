#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

WHEEL_SEPARATION = 0.233
WHEEL_RADIUS = 0.03575

class TwistToWheels(Node):
    def __init__(self):
        super().__init__('twist_to_wheels')
        self.pub = self.create_publisher(Float64MultiArray, 'wheel_velocity_controller/commands', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cb, 10)

    def cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        left = (v - w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        right = (v + w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        out = Float64MultiArray()
        out.data = [left, right]
        self.pub.publish(out)

def main():
    rclpy.init()
    node = TwistToWheels()
    rclpy.spin(node)

if __name__ == '__main__':
    main()