#!/usr/bin/env python3
"""Send the existing safe routes through Nav2 NavigateThroughPoses actions."""

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses


ROUTES = {
    'robot1': [(24.6, 2.0), (24.6, 13.1), (2.0, 13.1)],
    'robot2': [(25.6, 2.0), (25.6, 13.1), (18.0, 13.1), (18.0, 27.7)],
    'robot3': [(26.6, 2.0), (26.6, 13.1), (10.0, 13.1), (10.0, 40.0)],
    'robot4': [(27.6, 2.0), (27.6, 13.1), (6.0, 13.1), (6.0, 27.7)],
    'robot5': [(28.6, 2.0), (28.6, 13.1), (14.0, 13.1)],
}


def pose(node, namespace, x, y):
    stamped = PoseStamped()
    stamped.header.stamp = node.get_clock().now().to_msg()
    stamped.header.frame_id = f'{namespace}/map'
    stamped.pose.position.x = x
    stamped.pose.position.y = y
    stamped.pose.orientation.w = 1.0
    return stamped


def main():
    rclpy.init()
    node = rclpy.create_node('assign_nav2_safe_routes')
    clients = {}

    for namespace, route in ROUTES.items():
        client = ActionClient(node, NavigateThroughPoses, f'/{namespace}/navigate_through_poses')
        node.get_logger().info(f'Waiting for Nav2 action server of {namespace}...')
        if not client.wait_for_server(timeout_sec=90.0):
            node.get_logger().error(f'{namespace}: Nav2 action server is unavailable')
            continue
        goal = NavigateThroughPoses.Goal()
        goal.poses = [pose(node, namespace, x, y) for x, y in route[1:]]
        clients[namespace] = client.send_goal_async(goal)

    for namespace, future in clients.items():
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            node.get_logger().error(f'{namespace}: route was rejected')
        else:
            node.get_logger().info(f'{namespace}: Nav2 accepted safe route')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
