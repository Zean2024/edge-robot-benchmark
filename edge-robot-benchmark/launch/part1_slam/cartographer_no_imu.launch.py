import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    project_dir = os.path.join(
        os.path.expanduser('~'),
        'edge-robot-benchmark'
    )
    config_dir = os.path.join(
        project_dir, 'configs', 'part1_slam'
    )

    ekf_params = os.path.join(config_dir, 'ekf_slam_toolbox_no_imu.yaml')
    tf_filter_script = os.path.join(
        project_dir,
        'src',
        'part1_slam',
        'scripts',
        'tf_filter.py'
    )
    config_basename = 'cartographer_no_imu.lua'

    # Filter conflicting bag TF and publish scans only when the
    # required odom_combined -> laser transform is available.
    tf_filter = ExecuteProcess(
        cmd=['python3', tf_filter_script],
        output='screen',
        name='tf_filter_process',
    )

    # Provide the odometry TF required by the filter.
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

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_basename,
        ],
        remappings=[
            ('scan', '/scan_for_slam'),
            ('odom', '/odom'),
            ('imu', '/imu'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
