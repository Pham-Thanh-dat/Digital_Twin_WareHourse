#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf2_ros
from geometry_msgs.msg import TransformStamped

WHEEL_SEPARATION = 0.233
WHEEL_RADIUS = 0.03575

def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = 0.0
      
        self.last_time = None
        self.left_pos_prev = None
        self.right_pos_prev = None

        ns = self.get_namespace().strip('/')
        self.get_logger().info(f'DEBUG namespace = "{ns}"')
        self.prefix = f'{ns}/' if ns else ''
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.publish_tf)  # 100Hz
        self.sub = self.create_subscription(JointState, 'joint_states', self.cb, 10)

    def cb(self, msg):
        now = Time.from_msg(msg.header.stamp)
        if self.last_time is None:
            self.last_time = now
            try:
                li = msg.name.index('left_wheel_joint')
                ri = msg.name.index('right_wheel_joint')
                self.left_pos_prev = msg.position[li]
                self.right_pos_prev = msg.position[ri]
            except ValueError:
                return
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        try:
            li = msg.name.index('left_wheel_joint')
            ri = msg.name.index('right_wheel_joint')
        except ValueError:
            return

        left_pos = msg.position[li]
        right_pos = msg.position[ri]

        d_left = (left_pos - self.left_pos_prev) * WHEEL_RADIUS
        d_right = (right_pos - self.right_pos_prev) * WHEEL_RADIUS
        self.left_pos_prev = left_pos
        self.right_pos_prev = right_pos

        d_center = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / WHEEL_SEPARATION

        self.x += d_center * math.cos(self.yaw + d_yaw / 2.0)
        self.y += d_center * math.sin(self.yaw + d_yaw / 2.0)
        self.yaw += d_yaw

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.prefix + 'odom'
        odom.child_frame_id = self.prefix + 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_yaw / dt

        # Covariance: uoc luong do khong chac chan cua wheel odometry
        odom.pose.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05,
        ]
        odom.twist.covariance = [
            0.02, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.02, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.03,
        ]

        self.odom_pub.publish(odom)        
    def publish_tf(self):
        now = self.get_clock().now()
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.prefix + 'odom'
        t.child_frame_id = self.prefix + 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = yaw_to_quat(self.yaw)
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = WheelOdom()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
