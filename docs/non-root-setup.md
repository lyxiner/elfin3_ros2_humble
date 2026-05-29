# Running the Robot Arm Without Root

## Step 1: User group + udev rule

```bash
# 1.1 Create the ethercat group
sudo groupadd ethercat 2>/dev/null || echo "group already exists"

# 1.2 Add your user to the group (replace "xx" with your username)
sudo usermod -aG ethercat xx

# 1.3 Write a udev rule to grant raw access to the network interface
sudo tee /etc/udev/rules.d/99-ethercat.rules > /dev/null <<'EOF'
KERNEL=="enxc8a362e09547", GROUP="ethercat", MODE="0660"
EOF

# 1.4 Reload udev
sudo udevadm control --reload-rules
sudo udevadm trigger

# 1.5 Log out and log back in to apply the group membership
#     (opening a new terminal is NOT sufficient — you must re-login)
```

After re-login, verify:

```bash
groups | grep ethercat          # "ethercat" should appear in the list
ls -la /sys/class/net/enxc8a362e09547/   # group field should show "ethercat"
```

---

## Step 2: Set capabilities on `ros2_control_node`

```bash
# 2.1 Locate the actual binary path
which_node=$(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node
echo $which_node
# Expected: /opt/ros/humble/lib/controller_manager/ros2_control_node

# 2.2 Grant the required capabilities
sudo setcap cap_net_raw,cap_sys_nice+ep $which_node

# 2.3 Verify
getcap $which_node
# Expected output: .../ros2_control_node = cap_net_raw,cap_sys_nice+ep
```

---

## Step 3: Fix `LD_LIBRARY_PATH` being ignored after `setcap`

When a binary has capabilities set, the dynamic linker activates `AT_SECURE` mode and silently ignores `LD_LIBRARY_PATH`. The fix is to register the required library paths in `ld.so.conf` instead.

```bash
# 3.1 ROS Humble system library paths
sudo tee /etc/ld.so.conf.d/ros2-humble.conf > /dev/null <<'EOF'
/opt/ros/humble/lib
/opt/ros/humble/lib/x86_64-linux-gnu
/opt/ros/humble/opt/rviz_ogre_vendor/lib
/opt/ros/humble/opt/yaml_cpp_vendor/lib
EOF

# 3.2 Workspace package library paths (replace "xx_ws" with your workspace name)
sudo tee /etc/ld.so.conf.d/elfin3-ws.conf > /dev/null <<'EOF'
/home/xx/xx_ws/install/elfin_ros_control/lib
/home/xx/xx_ws/install/elfin_ethercat_driver/lib
/home/xx/xx_ws/install/elfin_basic_api/lib
/home/xx/xx_ws/install/soem_ros2/lib
/home/xx/xx_ws/install/elfin_robot_msgs/lib
EOF

# 3.3 Rebuild the linker cache
sudo ldconfig

# 3.4 Verify that libbackward is now visible to the system linker
ldconfig -p | grep libbackward
# Expected: libbackward.so (libc6,x86-64) => /opt/ros/humble/lib/libbackward.so
```

---

## Step 4: Test

Open a normal terminal (no `sudo`) and run:

```bash
source /opt/ros/humble/setup.bash
source ~/Git/elfin3_humble_ws/install/setup.bash
ros2 launch elfin_robot_bringup elfin_bringup.launch.py
```