#!/usr/bin/env python3

import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    HistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class TfFilter(Node):
    def __init__(self):
        super().__init__('tf_filter')

        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.tf_sub = self.create_subscription(
            TFMessage, '/tf_bag', self.tf_callback, tf_qos
        )
        self.tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)

        # Listen to the final TF tree, including /tf_static.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0), node=self)
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

        # The recorded base_footprint -> laser transform was constant
        # throughout all 463 observations in this bag.
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_laser_transform()

        # Gate incoming laser scans until their timestamp has a valid TF.
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan_for_slam', qos_profile_sensor_data
        )

        self.pending_scans = deque()
        self.scan_tf_wait_sec = 0.75
        self.max_pending_scans = 50

        self.tf_input_messages = 0
        self.tf_input_transforms = 0
        self.dropped_odom_base = 0
        self.dropped_dynamic_laser = 0
        self.forwarded_transforms = 0
        self.tf_output_messages = 0

        self.scans_received = 0
        self.scans_forwarded = 0
        self.scans_dropped_no_tf = 0
        self.scans_dropped_overflow = 0

        # This node uses wall time for its timer and wait timeout.
        self.stats_timer = self.create_timer(5.0, self.report_stats)
        self.scan_timer = self.create_timer(0.02, self.process_pending_scans)

        self.get_logger().info(
            'TF Filter active: filtering bag odometry TF, publishing the '
            'fixed laser transform on /tf_static, and gating /scan to '
            '/scan_for_slam until odom_combined -> laser is available.'
        )

    def publish_static_laser_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_footprint'
        transform.child_frame_id = 'laser'

        transform.transform.translation.x = 0.180
        transform.transform.translation.y = 0.000
        transform.transform.translation.z = 0.320

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.999999683
        transform.transform.rotation.w = 0.000796327

        self.static_broadcaster.sendTransform(transform)
        self.get_logger().info(
            'Published static transform base_footprint -> laser.'
        )

    def tf_callback(self, msg):
        self.tf_input_messages += 1
        self.tf_input_transforms += len(msg.transforms)
        filtered = []

        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')

            # EKF is the intended publisher of odom_combined -> base_footprint.
            if parent == 'odom_combined' and child == 'base_footprint':
                self.dropped_odom_base += 1
                continue

            # This fixed transform is supplied via /tf_static.
            if parent == 'base_footprint' and child == 'laser':
                self.dropped_dynamic_laser += 1
                continue

            filtered.append(transform)

        if filtered:
            output = TFMessage()
            output.transforms = filtered
            self.tf_pub.publish(output)
            self.tf_output_messages += 1
            self.forwarded_transforms += len(filtered)

    def scan_callback(self, msg):
        self.scans_received += 1

        if len(self.pending_scans) >= self.max_pending_scans:
            _, old_msg = self.pending_scans.popleft()
            self.scans_dropped_overflow += 1
            self.get_logger().warning(
                'Scan gate queue overflow: dropped scan at '
                f'{old_msg.header.stamp.sec}.'
                f'{old_msg.header.stamp.nanosec:09d}'
            )

        self.pending_scans.append((time.monotonic(), msg))

    def process_pending_scans(self):
        now = time.monotonic()

        # Preserve scan order. Wait briefly for TF messages to arrive.
        while self.pending_scans:
            received_at, msg = self.pending_scans[0]
            stamp = Time.from_msg(msg.header.stamp)

            try:
                available = self.tf_buffer.can_transform(
                    'odom_combined',
                    'laser',
                    stamp,
                    timeout=Duration(seconds=0.0),
                )
            except Exception as exc:
                available = False
                self.get_logger().debug(f'TF lookup not ready: {exc}')

            if available:
                self.scan_pub.publish(msg)
                self.scans_forwarded += 1
                self.pending_scans.popleft()
                continue

            age = now - received_at
            if age >= self.scan_tf_wait_sec:
                self.pending_scans.popleft()
                self.scans_dropped_no_tf += 1
                self.get_logger().warning(
                    'Explicitly dropped scan: no valid '
                    'odom_combined -> laser transform at stamp '
                    f'{msg.header.stamp.sec}.'
                    f'{msg.header.stamp.nanosec:09d} after '
                    f'{self.scan_tf_wait_sec:.2f}s '
                    f'(count={self.scans_dropped_no_tf}).'
                )
                continue

            # The oldest scan is not ready yet; retry on the next timer tick.
            break

    def report_stats(self):
        self.get_logger().info(
            'TF filter stats: '
            f'input_messages={self.tf_input_messages}, '
            f'input_transforms={self.tf_input_transforms}, '
            f'dropped_odom_base={self.dropped_odom_base}, '
            f'dropped_dynamic_laser={self.dropped_dynamic_laser}, '
            f'forwarded_transforms={self.forwarded_transforms}, '
            f'output_messages={self.tf_output_messages}, '
            f'scans_received={self.scans_received}, '
            f'scans_forwarded={self.scans_forwarded}, '
            f'scans_dropped_no_tf={self.scans_dropped_no_tf}, '
            f'scans_dropped_overflow={self.scans_dropped_overflow}, '
            f'scans_pending={len(self.pending_scans)}'
        )


def main():
    rclpy.init()
    node = None

    try:
        node = TfFilter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.report_stats()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
