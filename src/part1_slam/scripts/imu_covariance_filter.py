#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


# Initial estimate based on the M2DGR-plus IMU calibration.
# gyr_n is treated as a continuous-time gyro noise density.
# For a nominal 100 Hz IMU, use variance ~= gyr_n^2 / dt.
GYR_NOISE_DENSITY = 3.0032654435730201e-3
SAMPLE_RATE_HZ = 100.0
GYRO_VARIANCE = GYR_NOISE_DENSITY ** 2 * SAMPLE_RATE_HZ


class ImuCovarianceFilter(Node):
    def __init__(self):
        super().__init__('imu_covariance_filter')

        self.publisher = self.create_publisher(
            Imu,
            '/imu_ekf',
            10
        )

        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'IMU covariance adapter started; '
            f'publishing /imu_ekf, gyro variance={GYRO_VARIANCE:.8f}'
        )

    def callback(self, msg):
        # Copy the message so the original /imu stream is not modified.
        output = copy.deepcopy(msg)

        # Supply non-zero diagonal angular-velocity covariance.
        # The current EKF plan will fuse only angular_velocity.z.
        output.angular_velocity_covariance = [
            GYRO_VARIANCE, 0.0, 0.0,
            0.0, GYRO_VARIANCE, 0.0,
            0.0, 0.0, GYRO_VARIANCE
        ]

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
