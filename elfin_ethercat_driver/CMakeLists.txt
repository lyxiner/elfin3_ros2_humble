cmake_minimum_required(VERSION 3.16)
project(elfin_ethercat_driver)

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
find_package(soem_ros2 REQUIRED)
find_package(elfin_robot_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

set(DEPENDENCIES
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  trajectory_msgs
  tf2
  soem_ros2
  elfin_robot_msgs
)

# ----- shared library -----
add_library(${PROJECT_NAME}_lib SHARED
  src/elfin_ethercat_driver.cpp
  src/elfin_ethercat_client.cpp
  src/elfin_ethercat_io_client.cpp
  src/elfin_ethercat_manager.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib
  Boost::system
  Boost::thread
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

# ----- executable (standalone driver node) -----
add_executable(elfin_ethercat_driver_node src/elfin_ethercat_driver_node.cpp)
target_link_libraries(elfin_ethercat_driver_node
  ${PROJECT_NAME}_lib
  Boost::system
  Boost::thread
)
ament_target_dependencies(elfin_ethercat_driver_node ${DEPENDENCIES})

# ----- install -----
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS elfin_ethercat_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ----- export so downstream (elfin_ros_control) can find us -----
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()