cmake_minimum_required(VERSION 3.16)
project(elfin_basic_api)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----- find deps -----
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(elfin_robot_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  controller_manager_msgs
  moveit_core
  moveit_planners_ompl
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_eigen
  elfin_robot_msgs
)

# ----- library -----
add_library(${PROJECT_NAME}_lib SHARED
  src/elfin_basic_api.cpp
  src/elfin_teleop_api.cpp
  src/elfin_motion_api.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

# ----- executable -----
add_executable(elfin_basic_api_node src/elfin_basic_api_node.cpp)
target_link_libraries(elfin_basic_api_node ${PROJECT_NAME}_lib)
ament_target_dependencies(elfin_basic_api_node ${DEPENDENCIES})

# ----- install -----
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS elfin_basic_api_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ----- Python (the wx GUI script) -----
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/elfin_gui.py
  scripts/test.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY scripts/btn_icon
  DESTINATION lib/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()