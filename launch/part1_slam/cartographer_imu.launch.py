import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Resolve the repository root from this launch file so the stack can be
    # started from any checkout location (Windows workspace, WSL, container).
    project_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..')
    )

    config_dir = os.path.join(
        project_dir,
        'configs',
        'part1_slam'
    )

    # Under D1 Option B, the background EKF in this group uses the no-IMU
    # parameters (pure wheel odometry) so that IMU is consumed exactly once,
    # and only through Cartographer's native use_imu_data path.
    ekf_params = os.path.join(
        config_dir,
        'ekf_slam_toolbox_no_imu.yaml'
    )

    tf_filter_script = os.path.join(
        project_dir,
        'src',
        'part1_slam',
        'scripts',
        'tf_filter.py'
    )

    config_basename = 'cartographer_imu.lua'

    # Filter conflicting transforms and gate incoming laser scans.
    tf_filter = ExecuteProcess(
        cmd=['python3', tf_filter_script, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        name='tf_filter_process',
    )

    # Provide the odometry TF and /odometry/filtered topic consumed by
    # Cartographer. This EKF fuses wheel odometry only; IMU is not fused here.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Run Cartographer using filtered laser scans, the original IMU, and the
    # EKF output odometry. Cartographer consumes /odometry/filtered instead of
    # raw /odom so that its odometry input is semantically consistent with the
    # no-IMU group, and IMU enters only via use_imu_data = true in the Lua file.
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=[
            '-configuration_directory',
            config_dir,
            '-configuration_basename',
            config_basename,
        ],
        remappings=[
            ('scan', '/scan_for_slam'),
            ('odom', '/odometry/filtered'),
            ('imu', '/imu'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=['-resolution', '0.05'],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the ROS bag simulation clock',
        ),
        tf_filter,
        ekf_node,
        cartographer_node,
        occupancy_grid_node,
    ])