/*
 * Migrated to ROS2 Humble.
 *
 * Foxy -> Humble changes:
 *   - Removed include <controller_manager_msgs/msg/hardware_interface.hpp>:
 *     this message type was removed in Humble. The runtime info we used
 *     to read from there (HardwareInterface) is now in ListControllers'
 *     `claimed_interfaces` field, which the .cpp already uses.
 *   - Removed two declared-but-unused callbacks with old non-shared_ptr
 *     signatures (setVelocityScaling_cb, updateVelocityScaling_cb) to
 *     avoid future linker confusion.
 */

#ifndef ELFIN_BASIC_API_H
#define ELFIN_BASIC_API_H

#include <elfin_basic_api/elfin_teleop_api.h>
#include <elfin_basic_api/elfin_motion_api.h>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <elfin_robot_msgs/srv/set_string.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
// NOTE: controller_manager_msgs/msg/hardware_interface.hpp was removed in Humble.

#include <string.h>

namespace elfin_basic_api {

class ElfinBasicAPI
{
public:
  ElfinBasicAPI(const rclcpp::Node::SharedPtr& node,
                moveit::planning_interface::MoveGroupInterfacePtr& group,
                std::string action_name,
                planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);
  ~ElfinBasicAPI();

  rcl_interfaces::msg::SetParametersResult dynamicReconfigureCallback(
    const std::vector<rclcpp::Parameter> &parameters);

  void setVelocityScaling(double data);

  bool setRefLink_cb(
    const std::shared_ptr<elfin_robot_msgs::srv::SetString::Request> req,
    const std::shared_ptr<elfin_robot_msgs::srv::SetString::Response> resp);
  bool setEndLink_cb(
    const std::shared_ptr<elfin_robot_msgs::srv::SetString::Request> req,
    const std::shared_ptr<elfin_robot_msgs::srv::SetString::Response> resp);
  bool enableRobot_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
  bool disableRobot_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

  bool stopActCtrlrs(const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
  bool startElfinCtrlr(const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

  void set_vel_cb(const std_msgs::msg::Float32::SharedPtr msg);

private:
  moveit::planning_interface::MoveGroupInterfacePtr& group_;
  planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor_;

  rclcpp::Node::SharedPtr root_nh_;
  rclcpp::Node::SharedPtr local_nh_;

  ElfinTeleopAPI *teleop_api_ = nullptr;
  ElfinMotionAPI *motion_api_ = nullptr;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;
  control_msgs::action::FollowJointTrajectory::Goal goal_;

  double velocity_scaling_ = 0.4;

  rclcpp::Service<elfin_robot_msgs::srv::SetString>::SharedPtr set_ref_link_server_;
  rclcpp::Service<elfin_robot_msgs::srv::SetString>::SharedPtr set_end_link_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_robot_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr disable_robot_server_;

  std::string elfin_controller_init_name = "elfin_arm_controller";
  std::string elfin_controller_name_;
  std::vector<std::string> controller_joint_names_;

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr  list_controllers_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr get_motion_state_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr get_pos_align_state_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr raw_enable_robot_client_;

  std_srvs::srv::SetBool::Request::SharedPtr  raw_disable_robot_request_;
  std_srvs::srv::SetBool::Response::SharedPtr raw_disable_robot_response_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr raw_disable_robot_client_;

  std_msgs::msg::String ref_link_name_msg_;
  std_msgs::msg::String end_link_name_msg_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr get_vel;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ref_link_name_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr end_link_name_publisher_;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

}  // namespace elfin_basic_api

#endif