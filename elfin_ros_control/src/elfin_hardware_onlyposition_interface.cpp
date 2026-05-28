#include <elfin_ros_control/elfin_hw_onlyposition_interface.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <algorithm>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/handle.hpp>
#include <rclcpp/rclcpp.hpp>

namespace elfin_hardware_interface
{

using CallbackReturn = hardware_interface::CallbackReturn;

// ---------------- on_init ----------------
// Foxy 时代的 configure()，做参数解析、缓冲区分配、接口校验。
CallbackReturn ElfinHWInterface_PositoinOnly::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 节点提前创建好，read() 可能在 activate 之前被调用
  n_ = rclcpp::Node::make_shared("elfin_hw_position_only");

  // 从 URDF <hardware><param>...</param></hardware> 里取
  auto get_param = [&](const std::string & key, double default_val) {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) { return default_val; }
    try { return std::stod(it->second); }
    catch (const std::exception &) { return default_val; }
  };
  hw_start_sec_ = get_param("example_param_hw_start_duration_sec", 0.0);
  hw_stop_sec_  = get_param("example_param_hw_stop_duration_sec",  0.0);
  hw_slowdown_  = get_param("example_param_hw_slowdown",           1.0);

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  elfin_joint_positions_.assign(6, 0.0);
  elfin_joint_velocities_.assign(6, 0.0);
  elfin_joint_efforts_.assign(6, 0.0);
  elfin_ft_sensor_measurements_.assign(6, 0.0);
  elfin_tcp_pose_.assign(6, 0.0);
  elfin_position_commands_.assign(6, 0.0);
  elfin_position_commands_old_.assign(6, 0.0);
  elfin_velocity_commands_.assign(6, 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
        "Joint '%s' has '%s' command interface. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
        "Joint '%s' has %zu state interfaces. 1 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
        "Joint '%s' has '%s' state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

// ---------------- on_configure ----------------
CallbackReturn ElfinHWInterface_PositoinOnly::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 这里没有真硬件,只需要重置状态/命令缓冲
  for (uint i = 0; i < hw_states_.size(); ++i)
  {
    hw_states_[i]   = 0.0;
    hw_commands_[i] = 0.0;
  }
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface_PositoinOnly"), "Configured.");
  return CallbackReturn::SUCCESS;
}

// ---------------- export interfaces ----------------
std::vector<hardware_interface::StateInterface>
ElfinHWInterface_PositoinOnly::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ElfinHWInterface_PositoinOnly::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

// ---------------- on_activate (former start()) ----------------
CallbackReturn ElfinHWInterface_PositoinOnly::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
              "Starting... please wait...");
  for (int i = 0; i < static_cast<int>(hw_start_sec_); ++i)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
                "%.1f seconds left...", hw_start_sec_ - i);
  }

  // 把命令同步到状态:若 state 还是 NaN,给一个安全的 0;否则 cmd<-state。
  for (uint i = 0; i < hw_states_.size(); ++i)
  {
    if (std::isnan(hw_states_[i]))
    {
      hw_states_[i]   = 0.0;
      hw_commands_[i] = 0.0;
    }
    else
    {
      hw_commands_[i] = hw_states_[i];
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
              "System Successfully started!");
  return CallbackReturn::SUCCESS;
}

// ---------------- on_deactivate (former stop()) ----------------
CallbackReturn ElfinHWInterface_PositoinOnly::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
              "Stopping... please wait...");
  for (int i = 0; i < static_cast<int>(hw_stop_sec_); ++i)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
                "%.1f seconds left...", hw_stop_sec_ - i);
  }
  RCLCPP_INFO(rclcpp::get_logger("ElfinHWInterface_PositoinOnly"),
              "System successfully stopped!");
  return CallbackReturn::SUCCESS;
}

// ---------------- read ----------------
hardware_interface::return_type ElfinHWInterface_PositoinOnly::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(n_);

  // RRBot-style 仿真:让 state 立刻追上 command
  for (uint i = 0; i < hw_states_.size(); ++i)
  {
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]);
  }

  if (first_pass_ && !initialized_)
  {
    elfin_position_commands_     = hw_states_;
    elfin_position_commands_old_ = hw_states_;
    initialized_ = true;
    first_pass_  = false;
  }
  return hardware_interface::return_type::OK;
}

// ---------------- write ----------------
hardware_interface::return_type ElfinHWInterface_PositoinOnly::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 检测命令是否变化(原逻辑保留,留作调试入口)
  std::vector<double> pos_diff(hw_commands_.size(), 0.0);
  std::transform(
    hw_commands_.begin(), hw_commands_.end(),
    elfin_position_commands_old_.begin(),
    pos_diff.begin(),
    [](double a, double b) { return std::abs(a - b); });

  double pos_diff_sum = 0.0;
  for (double v : pos_diff) { pos_diff_sum += v; }

  if (pos_diff_sum != 0.0)
  {
    // 这里原来打印每个 joint 的命令(已注释),需要时可打开
  }
  elfin_position_commands_old_ = hw_commands_;
  return hardware_interface::return_type::OK;
}

}  // namespace elfin_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  elfin_hardware_interface::ElfinHWInterface_PositoinOnly,
  hardware_interface::SystemInterface)