# Elfin3 ROS2 Bringup & Troubleshooting

## Startup

```bash
# Terminal 1
sudo -i
ros2 launch elfin_robot_bringup elfin_bringup.launch.py

# Terminal 2
sudo -i
ros2 launch elfin3_ros2_moveit2 elfin3_moveit_rviz.launch.py

# Terminal 3
sudo -i
ros2 launch elfin_basic_api elfin_basic_api.launch.py

# Terminal 4
sudo -i
ros2 launch elfin_basic_api elfin_gui.launch.py
```

---

## Q1: Zero position is inaccurate — initial configuration does not match in RViz

**Fix:** Use the MoveIt GUI or RViz to jog the joints back to the home position, then manually align the motor angles to the zero position.

Next, read the current encoder positions from the motor (must also be run in a root terminal):

```bash
ros2 service call /get_current_position std_srvs/srv/SetBool "{data: true}"
```

The result is returned in `response.message`. The driver concatenates the two axis encoder counts of each EtherCAT slave into a string with the following format:

```
slave1_current_position: axis1: 123456, axis2: 654321. slave2_current_position: axis1: ...
```

> **Note:** These are raw `ACTPOSITION` encoder counts (type `int32_t`), not converted angles.

Then update `count_zeros` in `elfin_robot_bringup/config/elfin_drivers.yaml`.
**Important:** the order is `2 1 3 4 5 6`, matching:

```yaml
joint_names: [elfin_joint2, elfin_joint1, elfin_joint3, elfin_joint4, elfin_joint5, elfin_joint6]

count_zeros: [10244059, 10247140, 21865683, 24605305, 3310107, 3067334]
```

Apply the same change to `count_zeros` in `elfin_robot_bringup/config/elfin_arm_control.yaml`.

After both files are updated, rebuild and re-source:

```bash
colcon build
source install/setup.bash
```

---

## Q2: How to print the current joint positions?

```bash
ros2 topic echo /joint_states --once
```

---

## Q3: How to move the arm to a specific set of joint angles?

Use the action interface below. Modify `positions` and `sec` as needed. Alternatively, drag the joint sliders in MoveIt.

```bash
ros2 action send_goal /elfin_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [elfin_joint1, elfin_joint2, elfin_joint3, elfin_joint4, elfin_joint5, elfin_joint6],
      points: [{positions: [-0.5, -0.48, -2.1, 0.0, -0.9, -1.98], time_from_start: {sec: 8}}]
    }
  }"
```

---

## Q4: How to avoid running everything as root?

It is possible to run without `sudo` by configuring udev rules.
See [`non-root-setup.md`](docs/non-root-setup.md) for the setup procedure. After applying the rules, all commands can be run in a normal (non-root) terminal.