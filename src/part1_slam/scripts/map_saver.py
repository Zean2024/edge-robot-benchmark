#!/usr/bin/env python3
"""Save the latest occupancy grid as a PGM/YAML pair (map_server format).

Used after a run to capture the map produced by SLAM Toolbox, which does not
write a map file on its own. The node exits after the first map message.
"""

import os
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.declare_parameter('output_stem', 'map')
        self.declare_parameter('timeout_s', 30.0)
        self.output_stem = (
            self.get_parameter('output_stem').get_parameter_value().string_value
        )
        timeout = (
            self.get_parameter('timeout_s').get_parameter_value().double_value
        )
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self.callback, qos)
        self.deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        self.create_timer(1.0, self.check_timeout)
        self.saved = False

    def check_timeout(self):
        if self.get_clock().now().nanoseconds > self.deadline and not self.saved:
            self.get_logger().error('No /map message received before timeout.')
            rclpy.shutdown()

    def callback(self, msg):
        if self.saved:
            return
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        pixels = bytearray(width * height)
        for row in range(height):
            source_row = height - 1 - row
            for column in range(width):
                value = msg.data[source_row * width + column]
                if value < 0:
                    pixels[row * width + column] = 205
                elif value >= 65:
                    pixels[row * width + column] = 0
                else:
                    pixels[row * width + column] = 254

        stem = self.output_stem
        os.makedirs(os.path.dirname(os.path.abspath(stem)), exist_ok=True)
        with open(stem + '.pgm', 'wb') as handle:
            handle.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
            handle.write(bytes(pixels))
        with open(stem + '.yaml', 'w', encoding='utf-8') as handle:
            handle.write(
                f'image: {os.path.basename(stem)}.pgm\n'
                f'resolution: {resolution}\n'
                f'origin: [{origin.position.x}, {origin.position.y}, 0.0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
            )
        self.saved = True
        self.get_logger().info(f'Saved map {width}x{height} to {stem}.pgm')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MapSaver()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
