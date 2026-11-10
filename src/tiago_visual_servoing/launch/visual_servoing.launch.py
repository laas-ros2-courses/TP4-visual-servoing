import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    tiago_gazebo_dir = get_package_share_directory('tiago_gazebo')
    br2_gazebo_worlds_dir = get_package_share_directory('br2_gazebo_worlds')
    marker_sdf = os.path.join(br2_gazebo_worlds_dir, 'models', 'aruco_tag_60', 'model.sdf')

    tiago_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tiago_gazebo_dir, 'launch', 'tiago_gazebo.launch.py')
        ),
        launch_arguments={
            'is_public_sim': 'True',
            'world_name': 'empty',
        }.items(),
    )

    # Gazebo + controllers take a while to come up; give them a head start
    # before spawning the marker and starting the servoing nodes.
    spawn_marker = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-file', marker_sdf,
                    '-name', 'aruco_60',
                    '-x', '1.5', '-y', '0.0', '-z', '1.0',
                    '-P', '-1.5708',
                ],
                output='screen',
            )
        ],
    )

    servoing_nodes = TimerAction(
        period=12.0,
        actions=[
            Node(package='tiago_visual_servoing', executable='aruco_tracker', output='screen'),
            Node(package='tiago_visual_servoing', executable='head_servo', output='screen'),
        ],
    )

    return LaunchDescription([
        tiago_gazebo,
        spawn_marker,
        servoing_nodes,
    ])
