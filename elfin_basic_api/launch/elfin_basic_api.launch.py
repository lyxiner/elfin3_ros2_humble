#!/usr/bin/python3

# elfin_basic_api.launch.py (ROS2 Humble)
# Bug fix: original referenced elfin10_* packages, but this is the elfin3
# tree, so we now reference elfin3_ros2_gazebo / elfin3_ros2_moveit2.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return f.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('elfin3_ros2_gazebo'),
            'urdf', 'elfin3.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        'elfin3_ros2_moveit2', 'config/elfin3.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml(
        'elfin3_ros2_moveit2', 'config/kinematics.yaml')

    elfin_basic_api_node = Node(
        name='elfin_basic_node',
        package='elfin_basic_api',
        executable='elfin_basic_api_node',
        output='screen',
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )

    return LaunchDescription([
        elfin_basic_api_node,
    ])