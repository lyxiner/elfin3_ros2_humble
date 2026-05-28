#!/usr/bin/python3

# elfin_bringup.launch.py (ROS2 Humble)
#
# Bring up Elfin3 ros2_control stack. Same launch for fake and real hardware,
# switched by the `use_fake_hardware` argument:
#
#   ros2 launch elfin_robot_bringup elfin_bringup.launch.py             # real EtherCAT
#   ros2 launch elfin_robot_bringup elfin_bringup.launch.py \
#                use_fake_hardware:=true                                # mock_components
#
# Components:
#   - robot_state_publisher (with URDF processed for the chosen backend)
#   - ros2_control_node (loads ElfinHWInterface or mock_components via plugin)
#   - joint_state_broadcaster spawner
#   - elfin_arm_controller spawner
#
# Real-hardware note: requires CAP_NET_RAW for SOEM. Run once:
#   sudo setcap cap_net_raw+ep \
#     $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PythonExpression,
)


def generate_launch_description():

    # ---- launch args ----
    use_fake_arg = DeclareLaunchArgument(
        'use_fake_hardware', default_value='false',
        description='If true, use mock_components/GenericSystem instead of the EtherCAT plugin.',
    )
    use_fake = LaunchConfiguration('use_fake_hardware')

    # 同一个 xacro 通过两个互斥参数切换 backend,所以我们要在 launch 时
    # 把 use_real_hardware 算成 not(use_fake_hardware) 的字面字符串。
    use_real_str = PythonExpression(
        ["'false' if '", use_fake, "' == 'true' else 'true'"]
    )

    xacro_file = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'),
        'urdf', 'elfin3.urdf.xacro')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', xacro_file,
                ' use_fake_hardware:=', use_fake,
                ' use_real_hardware:=', use_real_str,
            ]),
            value_type=str,
        )
    }

    controllers_yaml = os.path.join(
        get_package_share_directory('elfin_robot_bringup'),
        'config', 'elfin_arm_control.yaml')

    # ---- nodes ----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output={'stdout': 'screen', 'stderr': 'screen'},
    )

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
        use_fake_arg,
        robot_state_publisher,
        ros2_control_node,

        # 串行启动:joint_state_broadcaster 先起,再起 elfin_arm_controller。
        # 避免 JTC 拿不到初始 joint state 的 race。
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_elfin_arm_controller],
            )
        ),
        load_joint_state_broadcaster,
    ])
