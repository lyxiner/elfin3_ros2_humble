#ifndef ELFIN_ROS2_CONTROL_HW_POSITIONONLY_INTERFACE
#define ELFIN_ROS2_CONTROL_HW_POSITIONONLY_INTERFACE

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <string>
#include <memory>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/visibility_control.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "visibility_control.h"

namespace elfin_hardware_interface
{

class ElfinHWInterface_PositoinOnly : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ElfinHWInterface_PositoinOnly)

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ELFIN_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // hardware parameters (parsed from URDF <ros2_control>)
  double hw_start_sec_  = 0.0;
  double hw_stop_sec_   = 0.0;
  double hw_slowdown_   = 1.0;

  rclcpp::Node::SharedPtr n_;

  bool first_pass_  = true;
  bool initialized_ = false;

  // state/command storage (one position state + one position command per joint)
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // legacy auxiliary buffers (kept for compatibility; not strictly necessary)
  std::vector<double> elfin_joint_positions_;
  std::vector<double> elfin_joint_velocities_;
  std::vector<double> elfin_joint_efforts_;
  std::vector<double> elfin_ft_sensor_measurements_;
  std::vector<double> elfin_tcp_pose_;
  std::vector<double> elfin_position_commands_;
  std::vector<double> elfin_position_commands_old_;
  std::vector<double> elfin_velocity_commands_;
};

}  // namespace elfin_hardware_interface

#endif