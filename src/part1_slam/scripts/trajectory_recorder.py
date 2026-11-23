#!/usr/bin/env python3
"""Record a SLAM trajectory as TUM (timestamp x y z qx qy qz qw).

Samples the map -> base_footprint transform during a run. TF is the only
trajectory source available for both SLAM Toolbox and Cartographer without
post-processing, so the same recorder serves all four experimental groups.

The file is written incrementally, so even a killed run leaves usable data.
"""

import os

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        self.declare_parameter('output_file', 'trajectory.tum')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('sample_period', 0.1)

        self.output_path = (
            self.get_parameter('output_file')
            .get_parameter_value()
            .string_value
        )
        self.map_frame = (
            self.get_parameter('map_frame').get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter('base_frame').get_parameter_value().string_value
        )
        self.sample_period = (
            self.get_parameter('sample_period').get_parameter_value().double_value
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0), node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.last_stamp_ns = -1
        self.samples = 0
        self.failures = 0

        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)
        self.file_handle = open(self.output_path, 'w', encoding='utf-8')

        self.get_logger().info(
            f'Recording {self.map_frame} -> {self.base_frame} to '
            f'{self.output_path} at {1.0 / self.sample_period:.1f} Hz'
        )
        self.timer = self.create_timer(self.sample_period, self.sample)

    def sample(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.failures += 1
            if self.failures % 20 == 1:
                self.get_logger().debug(f'TF not available yet: {exc}')
            return

        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= 0 or stamp_ns == self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns

        t = transform.transform.translation
        q = transform.transform.rotation
        self.file_handle.write(
            f'{stamp.sec}.{stamp.nanosec:09d} {t.x:.9f} {t.y:.9f} {t.z:.9f} '
            f'{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n'
        )
        self.samples += 1
        if self.samples % 50 == 0:
            self.file_handle.flush()

    def close(self):
        try:
            self.file_handle.flush()
            self.file_handle.close()
        except Exception:
            pass
        self.get_logger().info(
            f'Trajectory recorder finished: {self.samples} samples, '
            f'{self.failures} TF lookup misses'
        )


def main():
    rclpy.init()
    node = None
    try:
        node = TrajectoryRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
