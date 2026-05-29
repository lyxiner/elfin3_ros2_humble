```bash
# 终端1
sudo -i
ros2 launch elfin_robot_bringup elfin_bringup.launch.py
# 终端2
sudo -i
ros2 launch elfin3_ros2_moveit2 elfin3_moveit_rviz.launch.py
# 终端3
sudo -i
ros2 launch elfin_basic_api elfin_basic_api.launch.py
# 终端4
sudo -i
ros2 launch elfin_basic_api elfin_gui.launch.py
```
问题1：零位不准，在rviz中可以观察到初始构型对不上。

答：在gui/Rviz中使用moveit移动joints使得机械臂回到home位置，然后手动调节电机角度对准零位。然后在第二个【注意，需要也在root终端下】终端读此时电机编码器位置:

```bash
ros2 service call /get_current_position std_srvs/srv/SetBool "{data: true}"
```
返回值在 response.message 里。驱动实现会把每个从站的两个轴编码器计数拼成字符串返回，格式类似：

slave1_current_position:axis1: 123456, axis2: 654321. slave2_current_position: axis1: ...
这里拿到的是实际反馈的ACTPOSITION 编码器计数值，类型是 int32_t，不是换算后的角度。

然后在elfin_robot_bringup/config/elfin_drivers.yaml修改count_zeros ，注意顺序是 2 1 3 4 5 6。按照此顺序用读得的电机编码器值代替：

joint_names: [elfin_joint2, elfin_joint1, elfin_joint3, elfin_joint4, elfin_joint5, elfin_joint6] 

count_zeros: [10244059, 10247140, 21865683, 24605305, 3310107, 3067334]

同样修改elfin_robot_bringup/config/elfin_arm_control.yaml中对应的count_zeros。
都改完毕之后colcon build并且重新source。

问题2：如何打印各个关节的位置？

答：

```bash
ros2 topic echo /joint_states --once
```

问题3：执行使得机械臂移动到指定的角度？

答：如下。按需要修改positions和sec的值。或者在moveit中按照拖动角度条等，方法有很多。

```bash
ros2 action send_goal /elfin_arm_controller/follow_joint_trajectory   control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [elfin_joint1, elfin_joint2, elfin_joint3, elfin_joint4, elfin_joint5, elfin_joint6],
      points: [{positions: [-0.5, -0.48, -2.1, 0.0, -0.9, -1.98], time_from_start: {sec: 8}}]
    }
  }"
```

问题4：如何避免在root下启动？

答：


> 其实也可以不用root启动，但是需要设置一下udev规则……见《如何使得机械臂不必须工作在root下.md》然后就可以在普通终端中执行了
