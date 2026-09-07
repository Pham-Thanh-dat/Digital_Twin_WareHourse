#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class FixScanFrame(Node):
    def __init__(self):
        super().__init__('fix_scan_frame')
        ns = self.get_namespace().strip('/')
        self.prefix = f'{ns}/' if ns else ''
        self.pub = self.create_publisher(LaserScan, 'scan_fixed', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.cb, 10)

    def cb(self, msg):
        msg.header.frame_id = self.prefix + msg.header.frame_id
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FixScanFrame()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
