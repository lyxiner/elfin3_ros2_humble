cmake_minimum_required(VERSION 3.16)
project(elfin_ros_control)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)        # Humble 默认 C++17
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----- find dependencies -----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(elfin_ethercat_driver REQUIRED)   # 通过 ament 拿到 include + link 信息
find_package(elfin_robot_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

set(DEPENDENCIES
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  trajectory_msgs
  tf2
  hardware_interface
  controller_interface
  pluginlib
  elfin_ethercat_driver
  elfin_robot_msgs
)

# ----- the hardware_interface plugin library -----
add_library(elfin_hardware_interface SHARED
  src/elfin_hardware_interface.cpp
  src/elfin_hardware_onlyposition_interface.cpp
)

target_include_directories(elfin_hardware_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Boost 单独处理:elfin_hardware_interface.cpp 直接用了 boost::mutex/boost::thread
target_link_libraries(elfin_hardware_interface
  Boost::system
  Boost::thread
)

ament_target_dependencies(elfin_hardware_interface ${DEPENDENCIES})

# 让 pluginlib 找到插件描述
pluginlib_export_plugin_description_file(hardware_interface elfin_hardware_interface.xml)

# ----- install -----
install(
  TARGETS elfin_hardware_interface
  EXPORT export_elfin_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

# pluginlib 描述文件也要装到 share 里
install(
  FILES elfin_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# ----- export -----
ament_export_include_directories(include)
ament_export_libraries(elfin_hardware_interface)
ament_export_dependencies(${DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()