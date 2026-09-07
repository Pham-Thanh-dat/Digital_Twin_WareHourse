#!/usr/bin/env python3
"""Publish congestion zones as a Nav2-compatible OccupancyGrid overlay.

Each robot receives a grid with exactly the same geometry as its SLAM map.
Only congestion-zone cells are marked occupied; all other cells are unknown, so
the layer can be combined with the normal SLAM static layer without erasing
walls or shelves.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from multibot_interfaces.msg import CongestionZones


class CongestionCostmap(Node):
    def __init__(self):
        super().__init__('congestion_costmap')
        self.declare_parameter('robot_namespaces', ['robot1'])
        self.declare_parameter('zone_padding', 0.35)

        self.zone_padding = self.get_parameter('zone_padding').value
        self.zones = []
        self.maps = {}
        self.publisher_dict = {}

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            CongestionZones, '/congestion_zones', self.zones_callback, latched_qos)

        for namespace in self.get_parameter('robot_namespaces').value:
            self.publisher_dict[namespace] = self.create_publisher(
                OccupancyGrid, f'/{namespace}/congestion_costmap', latched_qos)
            self.create_subscription(
                OccupancyGrid,
                f'/{namespace}/map',
                lambda msg, ns=namespace: self.map_callback(ns, msg),
                latched_qos,
            )

    def zones_callback(self, msg):
        self.zones = [zone for zone in msg.zones if zone.active]
        self.publish_all()

    def map_callback(self, namespace, msg):
        self.maps[namespace] = msg
        self.publish_overlay(namespace)

    def publish_all(self):
        for namespace in self.maps:
            self.publish_overlay(namespace)

    def publish_overlay(self, namespace):
        source = self.maps.get(namespace)
        if source is None:
            return

        overlay = OccupancyGrid()
        overlay.header.stamp = self.get_clock().now().to_msg()
        overlay.header.frame_id = source.header.frame_id
        overlay.info = source.info

        width = source.info.width
        height = source.info.height
        resolution = source.info.resolution
        origin_x = source.info.origin.position.x
        origin_y = source.info.origin.position.y
        overlay.data = [-1] * (width * height)

        for zone in self.zones:
            radius = zone.radius + self.zone_padding
            min_col = max(0, math.floor((zone.x - radius - origin_x) / resolution))
            max_col = min(width - 1, math.ceil((zone.x + radius - origin_x) / resolution))
            min_row = max(0, math.floor((zone.y - radius - origin_y) / resolution))
            max_row = min(height - 1, math.ceil((zone.y + radius - origin_y) / resolution))

            for row in range(min_row, max_row + 1):
                cell_y = origin_y + (row + 0.5) * resolution
                for col in range(min_col, max_col + 1):
                    cell_x = origin_x + (col + 0.5) * resolution
                    if math.hypot(cell_x - zone.x, cell_y - zone.y) <= radius:
                        overlay.data[row * width + col] = 100

        self.publisher_dict[namespace].publish(overlay)


def main():
    rclpy.init()
    node = CongestionCostmap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
