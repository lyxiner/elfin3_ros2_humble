/*
 * Migrated to ROS2 Humble.
 *
 * Foxy -> Humble changes:
 *   - rclcpp::executor::ExecutorArgs() -> rclcpp::ExecutorOptions()
 *     (the rclcpp::executor namespace was deprecated and finally removed)
 *   - base_node added to the executor so its services/subs actually get
 *     serviced (previously rclcpp::spin(base_node) was called inline, but
 *     the constructor of ElfinBasicAPI also called rclcpp::spin() and
 *     blocked there forever).
 */

#include "elfin_basic_api/elfin_basic_api_node.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const std::string move_group_name = "elfin_arm";

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_nh = rclcpp::Node::make_shared("elfin_basic_api_node_base", node_options);
  auto base_node     = rclcpp::Node::make_shared("base_api_node",            node_options);

  // Humble: rclcpp::executor::ExecutorArgs() is gone, use ExecutorOptions().
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 5);
  executor.add_node(move_group_nh);
  executor.add_node(base_node);

  // Spin in a background thread so the constructor of ElfinBasicAPI can
  // talk to move_group / planning_scene_monitor while we set things up.
  std::thread spin_thread([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterfacePtr move_group(
    new moveit::planning_interface::MoveGroupInterface(move_group_nh, move_group_name));

  move_group->startStateMonitor(2.0);
  move_group->getCurrentJointValues();

  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
    new robot_model_loader::RobotModelLoader(move_group_nh, "robot_description"));

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
    new planning_scene_monitor::PlanningSceneMonitor(move_group_nh, robot_model_loader));

  planning_scene_monitor->startStateMonitor("/joint_states");
  planning_scene_monitor->startWorldGeometryMonitor();
  planning_scene_monitor->setPlanningScenePublishingFrequency(25);
  planning_scene_monitor->startPublishingPlanningScene(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
    "/elfin_basic_api/publish_planning_scene");
  planning_scene_monitor->startSceneMonitor();

  elfin_basic_api::ElfinBasicAPI basic_api(
    base_node, move_group,
    "elfin_arm_controller/follow_joint_trajectory",
    planning_scene_monitor);

  // executor 在后台线程 spin,主线程等它退出。
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}