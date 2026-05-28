#!/usr/bin/python3

# elfin3_moveit_rviz.launch.py (ROS2 Humble)
#
# Pure MoveIt2 + RViz launch. Does NOT start a controller_manager and does
# NOT spawn any controllers - those must come from whoever provides the
# hardware:
#
#   * Gazebo:  ros2 launch elfin3_ros2_gazebo elfin3_gazebo.launch.py
#              then this launch with use_sim_time:=true
#   * Real:    ros2 launch elfin_robot_bringup elfin_bringup.launch.py
#              then this launch with use_sim_time:=false (default)
#
# Why use_sim_time matters: gazebo_ros2_control publishes /joint_states with
# simulation time stamps (starting from 0 at gz launch). If move_group runs
# on wall clock, it compares wall-clock now() with sim-clock joint_state
# stamp, sees a multi-billion second diff, and decides it has "no recent
# robot state" -> Execute aborts even though Plan succeeded.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration, Command, FindExecutable,
)
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
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

    # ---- launch args ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use Gazebo simulation clock (set true when used with elfin3_gazebo.launch.py).',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz_file', default_value='False', description='If True, skip launching RViz.',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    load_rviz    = LaunchConfiguration('rviz_file')

    # ---- planning context ----
    xacro_file = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'),
        'urdf', 'elfin3.urdf.xacro')
    # MoveIt only consumes geometry/kinematics from URDF, so the choice of
    # use_real_hardware vs use_fake_hardware here is irrelevant. We pass
    # use_real_hardware:=true purely to keep it parseable when elfin_description
    # meshes and the hardware plugin are both installed.
    robot_description_cmd = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_fake_hardware:=false',
        ' use_real_hardware:=true',
    ])
    robot_description = {'robot_description': robot_description_cmd}

    robot_description_semantic_config = load_file(
        'elfin3_ros2_moveit2', 'config/elfin3.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml(
        'elfin3_ros2_moveit2', 'config/kinematics.yaml')

    # ---- OMPL planning pipeline ----
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'elfin3_ros2_moveit2', 'config/ompl_planning.yaml')
    if ompl_planning_yaml:
        ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # ---- MoveIt simple controller manager ----
    moveit_simple_controllers_yaml = load_yaml(
        'elfin3_ros2_moveit2', 'config/elfin_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.05,   # 提高容忍以减少 fp 误判
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ---- move_group ----
    # 关键: use_sim_time 透传到 move_group,这样它用 sim clock 跟 /joint_states 时间戳对得上。
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---- RViz ----
    rviz_full_config = os.path.join(
        get_package_share_directory('elfin3_ros2_moveit2'), 'launch',
        'elfin3_moveit2.rviz')
    rviz_node_full = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {'use_sim_time': use_sim_time},
        ],
        condition=UnlessCondition(load_rviz),
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        run_move_group_node,
        rviz_node_full,
    ])