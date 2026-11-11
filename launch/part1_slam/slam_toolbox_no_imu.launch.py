import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    project_dir = os.path.join(
        os.path.expanduser('~'),
        'edge-robot-benchmark'
    )

    params_file = os.path.join(
        project_dir,
        'configs',
        'part1_slam',
        'slam_toolbox_no_imu.yaml'
    )

    ekf_params = os.path.join(
        project_dir,
        'configs',
        'part1_slam',
        'ekf_slam_toolbox_no_imu.yaml'
    )

    tf_filter_script = os.path.join(
        project_dir,
        'src',
        'part1_slam',
        'scripts',
        'tf_filter.py'
    )

    # Relay bag TF while dropping the conflicting odometry transform.
    tf_filter = ExecuteProcess(
        cmd=['python3', tf_filter_script, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        name='tf_filter_process',
    )

    # EKF without IMU input.
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

    # SLAM Toolbox lifecycle node.
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'use_lifecycle_manager': False,
            },
        ],
    )

    # Automatically configure SLAM Toolbox after launch.
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate only after the configure transition succeeds.
    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(
                    msg='[SLAM Toolbox] Configured; activating node.'
                ),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the ROS bag simulation clock',
        ),
        tf_filter,
        ekf_node,
        activate_handler,
        slam_node,
        configure_event,
    ])