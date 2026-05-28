#!/usr/bin/python3

# elfin3_moveit.launch.py (ROS2 Humble, real hardware)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():

    # planning context
    xacro_file = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'),
        'urdf', 'elfin3.urdf.xacro')

    robot_description_cmd = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file,
         ' use_fake_hardware:=false',
         ' use_real_hardware:=true'])

    robot_description = {
        'robot_description': ParameterValue(robot_description_cmd, value_type=str)
    }

    # static tf
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'elfin_base_link'],
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ros2_control node (real hardware via EtherCAT)
    ros2_controllers_path = os.path.join(
        get_package_share_directory('elfin_robot_bringup'),
        'config', 'elfin_arm_control.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        output={'stdout': 'screen', 'stderr': 'screen'},
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        ros2_control_node,
    ])
