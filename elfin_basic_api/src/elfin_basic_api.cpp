/*
 * Migrated to ROS2 Humble.
 *
 * Foxy -> Humble changes in this file:
 *   - set_on_parameters_set_callback()      -> add_on_set_parameters_callback()
 *   - SwitchController: start_controllers   -> activate_controllers
 *                       stop_controllers    -> deactivate_controllers
 *   - ListControllers::state "running"      -> "active"
 *   - removed blocking rclcpp::spin() from the constructor
 *     (spinning is done by the executor in elfin_basic_api_node.cpp)
 *   - declare_parameter() for parameters used by get_parameter_or()
 *   - lazy-init raw_disable_robot_request_/response_ shared_ptrs
 */

#include <elfin_basic_api/elfin_basic_api.h>

namespace elfin_basic_api {

ElfinBasicAPI::ElfinBasicAPI(const rclcpp::Node::SharedPtr& node,
                             moveit::planning_interface::MoveGroupInterfacePtr& group,
                             std::string action_name,
                             planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : group_(group),
    planning_scene_monitor_(planning_scene_monitor),
    local_nh_(node)
{
  teleop_api_ = new ElfinTeleopAPI(local_nh_, group, action_name, planning_scene_monitor);
  motion_api_ = new ElfinMotionAPI(local_nh_, group, planning_scene_monitor);

  // Humble: set_on_parameters_set_callback() was removed in Galactic+.
  // Use add_on_set_parameters_callback() (same signature, different name).
  local_nh_->add_on_set_parameters_callback(
    std::bind(&ElfinBasicAPI::dynamicReconfigureCallback, this, std::placeholders::_1));

  set_ref_link_server_ = local_nh_->create_service<elfin_robot_msgs::srv::SetString>(
    "set_reference_link",
    std::bind(&ElfinBasicAPI::setRefLink_cb, this, std::placeholders::_1, std::placeholders::_2));
  set_end_link_server_ = local_nh_->create_service<elfin_robot_msgs::srv::SetString>(
    "set_end_link",
    std::bind(&ElfinBasicAPI::setEndLink_cb, this, std::placeholders::_1, std::placeholders::_2));
  enable_robot_server_ = local_nh_->create_service<std_srvs::srv::SetBool>(
    "/elfin_basic_api/enable_robot",
    std::bind(&ElfinBasicAPI::enableRobot_cb, this, std::placeholders::_1, std::placeholders::_2));
  disable_robot_server_ = local_nh_->create_service<std_srvs::srv::SetBool>(
    "/elfin_basic_api/disable_robot",
    std::bind(&ElfinBasicAPI::disableRobot_cb, this, std::placeholders::_1, std::placeholders::_2));

  // 显式 declare 参数,避免 Humble 默认设置下 get_parameter_or 抛 ParameterNotDeclaredException
  if (!local_nh_->has_parameter("controller_name")) {
    local_nh_->declare_parameter<std::string>("controller_name", elfin_controller_init_name);
  }
  if (!local_nh_->has_parameter("velocity_scaling")) {
    local_nh_->declare_parameter<double>("velocity_scaling", 0.4);
  }

  local_nh_->get_parameter_or("controller_name", elfin_controller_name_, elfin_controller_init_name);

  switch_controller_client_ =
    local_nh_->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
  list_controllers_client_ =
    local_nh_->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
  get_motion_state_client_    = local_nh_->create_client<std_srvs::srv::SetBool>("/get_motion_state");
  get_pos_align_state_client_ = local_nh_->create_client<std_srvs::srv::SetBool>("/get_pos_align_state");
  raw_enable_robot_client_    = local_nh_->create_client<std_srvs::srv::SetBool>("/enable_robot");
  raw_disable_robot_client_   = local_nh_->create_client<std_srvs::srv::SetBool>("/disable_robot");

  // 原代码这两个 shared_ptr 声明了但从未 make_shared,disableRobot_cb 里直接 -> 用会段错误
  raw_disable_robot_request_  = std::make_shared<std_srvs::srv::SetBool::Request>();
  raw_disable_robot_response_ = std::make_shared<std_srvs::srv::SetBool::Response>();

  ref_link_name_publisher_ = local_nh_->create_publisher<std_msgs::msg::String>("reference_link_name", 1);
  end_link_name_publisher_ = local_nh_->create_publisher<std_msgs::msg::String>("end_link_name", 1);

  ref_link_name_msg_.data = group_->getPlanningFrame();
  end_link_name_msg_.data = group_->getEndEffectorLink();
  ref_link_name_publisher_->publish(ref_link_name_msg_);
  end_link_name_publisher_->publish(end_link_name_msg_);

  get_vel = local_nh_->create_subscription<std_msgs::msg::Float32>(
    "vel", rclcpp::SensorDataQoS(),
    std::bind(&ElfinBasicAPI::set_vel_cb, this, std::placeholders::_1));

  double vel = 0.4;
  local_nh_->get_parameter("velocity_scaling", vel);
  velocity_scaling_ = vel;

  tfBuffer      = std::make_unique<tf2_ros::Buffer>(local_nh_->get_clock());
  tf_listener_  = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  // 关键修复:不再在构造函数里调用 rclcpp::spin(local_nh_)。
  // spin 由 elfin_basic_api_node.cpp 的 executor 负责;在这里 spin 会让构造函数
  // 永不返回,后续主函数里第二次 spin(base_node) 永远到不了。
}

ElfinBasicAPI::~ElfinBasicAPI()
{
  if (teleop_api_ != nullptr) { delete teleop_api_; teleop_api_ = nullptr; }
  if (motion_api_ != nullptr) { delete motion_api_; motion_api_ = nullptr; }
}

rcl_interfaces::msg::SetParametersResult ElfinBasicAPI::dynamicReconfigureCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &parameter : parameters) {
    if (parameter.get_name() == "velocity_scaling" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      setVelocityScaling(parameter.as_double());
      RCLCPP_INFO(local_nh_->get_logger(),
                  "Parameter 'velocity_scaling' changed: %f", parameter.as_double());
    }
  }
  return result;
}

void ElfinBasicAPI::set_vel_cb(const std_msgs::msg::Float32::SharedPtr msg)
{
  setVelocityScaling(static_cast<double>(msg->data));
}

void ElfinBasicAPI::setVelocityScaling(double data)
{
  velocity_scaling_ = data;
  teleop_api_->setVelocityScaling(velocity_scaling_);
  motion_api_->setVelocityScaling(velocity_scaling_);
}

bool ElfinBasicAPI::setRefLink_cb(
  const std::shared_ptr<elfin_robot_msgs::srv::SetString::Request> req,
  const std::shared_ptr<elfin_robot_msgs::srv::SetString::Response> resp)
{
  if (!tfBuffer->_frameExists(req->data)) {
    resp->success = false;
    resp->message = std::string("There is no frame named ") + req->data;
    return true;
  }
  teleop_api_->setRefFrames(req->data);
  motion_api_->setRefFrames(req->data);
  ref_link_name_msg_.data = req->data;
  ref_link_name_publisher_->publish(ref_link_name_msg_);
  resp->success = true;
  resp->message = "Setting reference link succeeded";
  return true;
}

bool ElfinBasicAPI::setEndLink_cb(
  const std::shared_ptr<elfin_robot_msgs::srv::SetString::Request> req,
  const std::shared_ptr<elfin_robot_msgs::srv::SetString::Response> resp)
{
  if (!tfBuffer->_frameExists(req->data)) {
    resp->success = false;
    resp->message = std::string("There is no frame named ") + req->data;
    return true;
  }
  teleop_api_->setEndFrames(req->data);
  motion_api_->setEndFrames(req->data);
  end_link_name_msg_.data = req->data;
  end_link_name_publisher_->publish(end_link_name_msg_);
  resp->success = true;
  resp->message = "Setting end link succeeded";
  return true;
}

bool ElfinBasicAPI::enableRobot_cb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
  if (!req->data) {
    resp->success = false;
    resp->message = "request's data is false";
    return true;
  }

  if (!raw_enable_robot_client_->service_is_ready()) {
    resp->message = "there is no real driver running";
    resp->success = false;
    return true;
  }

  auto req_tmp  = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto resp_tmp = std::make_shared<std_srvs::srv::SetBool::Response>();

  if (!stopActCtrlrs(resp_tmp)) {
    resp->success = resp_tmp->success;
    resp->message = resp_tmp->message;
    return true;
  }

  if (!get_motion_state_client_->service_is_ready()) {
    resp->message = "there is no get_motion_state service";
    resp->success = false;
    return true;
  }
  req_tmp->data = true;
  auto motion_state_fut = get_motion_state_client_->async_send_request(req_tmp);
  resp_tmp = motion_state_fut.get();
  if (resp_tmp->success) {
    resp->message = "failed to enable the robot, it's moving";
    resp->success = false;
    return true;
  }

  if (!get_pos_align_state_client_->service_is_ready()) {
    resp->message = "there is no get_pos_align_state service";
    resp->success = false;
    return true;
  }
  req_tmp->data = true;
  auto pos_align_fut = get_pos_align_state_client_->async_send_request(req_tmp);
  resp_tmp = pos_align_fut.get();
  if (!resp_tmp->success) {
    resp->message = "failed to enable the robot, commands aren't aligned with actual positions";
    resp->success = false;
    return true;
  }

  if (!raw_enable_robot_client_->service_is_ready()) {
    resp->message = "there is no real driver running";
    resp->success = false;
    return true;
  }

  auto raw_enable_req  = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto raw_enable_resp = std::make_shared<std_srvs::srv::SetBool::Response>();
  raw_enable_req->data = true;
  auto raw_enable_fut  = raw_enable_robot_client_->async_send_request(raw_enable_req);
  raw_enable_resp      = raw_enable_fut.get();
  resp->message = "robot is enabled";
  resp->success = true;

  if (!startElfinCtrlr(resp_tmp)) {
    resp->message = resp_tmp->message;
    resp->success = resp_tmp->success;
    return true;
  }
  return true;
}

bool ElfinBasicAPI::disableRobot_cb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
  if (!req->data) {
    resp->success = false;
    resp->message = "request's data is false";
    return true;
  }
  if (!raw_disable_robot_client_->service_is_ready()) {
    resp->message = "there is no real driver running";
    resp->success = false;
    return true;
  }
  raw_disable_robot_request_->data = true;
  auto raw_disable_fut = raw_disable_robot_client_->async_send_request(raw_disable_robot_request_);
  raw_disable_robot_response_ = raw_disable_fut.get();
  resp->message = raw_disable_robot_response_->message;
  resp->success = raw_disable_robot_response_->success;
  return true;
}

bool ElfinBasicAPI::stopActCtrlrs(const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
  if (!list_controllers_client_->service_is_ready()) {
    resp->message = "there is no controller manager";
    resp->success = false;
    return false;
  }

  auto list_req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto list_fut = list_controllers_client_->async_send_request(list_req);
  auto list_resp = list_fut.get();

  std::vector<std::string> controllers_to_stop;
  controller_joint_names_.clear();

  // 收集 elfin 控制器占用的 joint interface 名
  for (const auto & ctrl : list_resp->controller) {
    if (strcmp(ctrl.name.c_str(), elfin_controller_name_.c_str()) == 0) {
      for (const auto & resrc : ctrl.claimed_interfaces) {
        controller_joint_names_.push_back(resrc);
      }
      break;
    }
  }

  // 找出"正在 active"且声明的资源跟 elfin 控制器不一致的控制器
  // Humble: state 字段值由 "running" 改为 "active"
  for (const auto & ctrl : list_resp->controller) {
    if (strcmp(ctrl.state.c_str(), "active") == 0) {
      bool break_flag = false;
      for (const auto & resrc : ctrl.claimed_interfaces) {
        for (const auto & joint_name : controller_joint_names_) {
          if (strcmp(resrc.c_str(), joint_name.c_str()) != 0) {
            break_flag = true;
            controllers_to_stop.push_back(ctrl.name);
          }
          if (break_flag) { break; }
        }
        if (break_flag) { break; }
      }
    }
  }

  if (!controllers_to_stop.empty()) {
    if (!switch_controller_client_->service_is_ready()) {
      resp->message = "there is no controller manager";
      resp->success = false;
      return false;
    }
    auto sw_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    // Humble: start_controllers/stop_controllers -> activate_controllers/deactivate_controllers
    sw_req->activate_controllers.clear();
    sw_req->deactivate_controllers = controllers_to_stop;
    sw_req->strictness = sw_req->STRICT;

    auto sw_fut = switch_controller_client_->async_send_request(sw_req);
    auto sw_resp = sw_fut.get();
    if (!sw_resp->ok) {
      resp->message = "Failed to stop active controllers";
      resp->success = false;
      return false;
    }
  }
  return true;
}

bool ElfinBasicAPI::startElfinCtrlr(const std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
  if (!switch_controller_client_->service_is_ready()) {
    resp->message = "there is no controller manager";
    resp->success = false;
    return false;
  }
  auto sw_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  // Humble field names:
  sw_req->activate_controllers.clear();
  sw_req->activate_controllers.push_back(elfin_controller_name_);
  sw_req->deactivate_controllers.clear();
  sw_req->strictness = sw_req->STRICT;

  auto sw_fut = switch_controller_client_->async_send_request(sw_req);
  auto sw_resp = sw_fut.get();
  if (!sw_resp->ok) {
    resp->message = "Failed to start the default controller";
    resp->success = false;
    return false;
  }
  return true;
}

}  // namespace elfin_basic_api