#!/usr/bin/python3

# elfin3.launch.py:
# Launch file for the elfin3 Robot GAZEBO + MoveIt!2 SIMULATION in ROS2 Humble.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
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

    # *********************** Gazebo *********************** #
    elfin3_world = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'),
        'worlds', 'elfin3.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
             '/gazebo.launch.py']),
        launch_arguments={'world': elfin3_world}.items(),
    )

    # ***** ROBOT DESCRIPTION ***** #
    elfin3_description_path = get_package_share_directory('elfin3_ros2_gazebo')
    xacro_file = os.path.join(elfin3_description_path, 'urdf', 'elfin3.urdf.xacro')

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file,
         ' use_fake_hardware:=false',
         ' use_real_hardware:=false'])

    robot_description = {
        'robot_description': ParameterValue(robot_description_config, value_type=str)
    }

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'elfin3',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )

    # ***** STATIC TF & STATE PUBLISHER ***** #
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'elfin_base_link'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # ***** CONTROLLERS (Humble: spawner, not spawner.py) ***** #
    ros2_controllers_path = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'),
        'config', 'elfin_arm_controller.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        output={'stdout': 'screen', 'stderr': 'screen'},
    )

    # 用 Node + spawner 取代旧的 `ros2 control load_start_controller`
    # 注意:joint_state_controller -> joint_state_broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    load_elfin_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['elfin_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # *********************** MoveIt!2 *********************** #
    rviz_arg = DeclareLaunchArgument(
        'rviz_file', default_value='False', description='Load RVIZ file.')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # planning context
    robot_description_xacro = xacro.process_file(xacro_file).toxml()
    robot_description_full = {'robot_description': robot_description_xacro}

    robot_description_semantic_config = load_file(
        'elfin3_ros2_moveit2', 'config/elfin3.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('elfin3_ros2_moveit2', 'config/kinematics.yaml')

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
    ompl_planning_yaml = load_yaml('elfin3_ros2_moveit2', 'config/ompl_planning.yaml')
    if ompl_planning_yaml:
        ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

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
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description_full,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    load_RVIZfile = LaunchConfiguration('rviz_file')
    rviz_base = os.path.join(get_package_share_directory('elfin3_ros2_moveit2'), 'launch')
    rviz_full_config = os.path.join(rviz_base, 'elfin3_moveit2.rviz')
    rviz_node_full = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description_full,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=UnlessCondition(load_RVIZfile),
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        static_tf,
        robot_state_publisher,
        ros2_control_node,

        # Gazebo 把模型 spawn 完之后再启动控制器与 MoveIt
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_broadcaster,
                    load_elfin_arm_controller,
                    rviz_arg,
                    declare_use_sim_time_cmd,
                    rviz_node_full,
                    run_move_group_node,
                ]
            )
        ),
    ])
