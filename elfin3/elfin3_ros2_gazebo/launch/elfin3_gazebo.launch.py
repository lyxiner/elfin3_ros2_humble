#!/usr/bin/python3

# elfin3_gazebo.launch.py (ROS2 Humble)
#
# Migration + bug-fix summary:
#   - `ros2 control load_start_controller` was removed in Humble.
#     Use Node + spawner executable instead.
#   - 'joint_state_controller' -> 'joint_state_broadcaster' (controller NAME).
#   - The original passed 'joint_trajectory_controller' as the controller name
#     for the second spawner, but that is the TYPE; the actual controller name
#     defined in elfin_arm_controller.yaml is 'elfin_arm_controller'.
#   - The original never added the controller-loading actions to the returned
#     LaunchDescription, so Gazebo came up with no controllers at all. Fixed.
#   - Wait for the entity to be spawned before starting controllers
#     (controller_manager only exists once gazebo_ros2_control plugin loads,
#     which happens after spawn_entity finishes).

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import (
    IncludeLaunchDescription, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable


def generate_launch_description():

    # Gazebo classic
    world_path = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'),
        'worlds', 'elfin3.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'),
                          'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_path}.items(),
    )

    # robot_description from xacro
    xacro_file = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'),
        'urdf', 'elfin3.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', xacro_file,
                ' use_fake_hardware:=false',
                ' use_real_hardware:=false',
            ]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'elfin3',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )

    # Controller spawners (Humble: use Node + spawner executable, by NAME)
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    load_elfin_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['elfin_arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,

        # Only spawn controllers after the entity (and therefore the
        # gazebo_ros2_control plugin + controller_manager) is alive.
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_broadcaster,
                    load_elfin_arm_controller,
                ],
            )
        ),
    ])
