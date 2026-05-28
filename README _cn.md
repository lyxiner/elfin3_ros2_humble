偶尔会出现零位不准的情况，在rviz中可以观察到构型对不上，此时在gui中让机械臂回到home位置，然后手动调节电机角度对准零位 
## 在第二个【注意，需要也在root终端下】终端读此时电机编码器位置 
```bash
ros2 service call /get_current_position std_srvs/srv/SetBool "{data: true}"
```
返回值在 response.message 里。驱动实现会把每个从站的两个轴编码器计数拼成字符串返回，格式类似：

  slave1_current_position:
  axis1: 123456, axis2: 654321.
  slave2_current_position:
  axis1: ...
这里拿到的是实际反馈的 ACTPOSITION 编码器计数值，类型是 int32_t，不是换算后的角度。
## 否则，会出现
waiting for service to become available...
这个问题困扰好久，还以为是哪里不对，服务没启用
按照以下思路才排查掉：
ros2 service list -t | grep -E 'get_current_position|get_txpdo|get_rxpdo|enable_robot'

ros2 node list

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"

ps -ef | grep -E 'elfin_ethercat|ros2_control_node|elfin_basic_api' | grep -v grep
ros2 service list 里如果有 */get_current_position，那就是名字不对，直接用完整服务名调用。
  - ros2 node list 有很多节点但没有这个服务，说明当前启动方式没有暴露这个服务。
  - ros2 node list 也几乎是空的，优先怀疑 source 环境或 ROS_DOMAIN_ID 不一致。
最后实际运行发现
  1. 当前这个终端根本没有看到任何 ROS 2 节点，ros2 node list 是空的。
  2. 但实际 ROS 进程是存在的，而且都跑在 root 下面。
  3. 所以 waiting for service to become available... 不是服务卡住，而是你这个终端不在同一个 ROS graph 里。
最后，切换到root下成功解决

然后在/home/sicheng/Github/ROS2/elfin3_ws/src/elfin_robot_ros2/elfin_robot_bringup/config/elfin_drivers.yaml 修改count_zeros ，注意顺序是 2 1 3456 joint_names: [elfin_joint2, elfin_joint1, elfin_joint3, elfin_joint4, elfin_joint5, elfin_joint6] count_zeros: [10244059, 10247140, 21865683, 24605305, 3310107, 3067334]
同样修改/home/sicheng/Github/ROS2/elfin3_ws/src/elfin_robot_ros2/elfin_robot_bringup/config/elfin_arm_control.yaml中对应的count_zeros
都改完毕之后colcon build并且重新source install/setup.bash

sudo -i

ros2 launch elfin_robot_bringup elfin_bringup.launch.py

ros2 launch elfin3_ros2_moveit2 elfin3_moveit_rviz.launch.py

ros2 launch elfin_basic_api elfin_basic_api.launch.py

ros2 launch elfin_basic_api elfin_gui.launch.py

> 其实也可以不用root启动，但是需要设置一下udev规则……见《如何使得机械臂不必须工作在root下.md》然后就可以在普通终端中执行了

打印各个关节的位置:
ros2 topic echo /joint_states --once

下面可以使得每一个关节都移动到指定的位置
root@wz-X299-WU8:/home/wz/Git/elfin3_humble_ws# ros2 action send_goal /elfin_arm_controller/follow_joint_trajectory   control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [elfin_joint1, elfin_joint2, elfin_joint3, elfin_joint4, elfin_joint5, elfin_joint6],
      points: [{positions: [-0.5, -0.48, -2.1, 0.0, -0.9, -1.98], time_from_start: {sec: 8}}]
    }
  }"