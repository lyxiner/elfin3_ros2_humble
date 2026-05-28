cmake_minimum_required(VERSION 3.16)
project(soem_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)

if(WIN32)
  set(OS "win32")
else()
  set(OS "linux")
endif()

# explicitly find pthread to be able to explicitly export the pthread dependency
set(PTHREAD_LIB "")
find_package(Threads)
if(CMAKE_THREAD_LIBS_INIT)
  string(LENGTH ${CMAKE_THREAD_LIBS_INIT} _length)
  if(_length GREATER 2)
    string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 0 2 _prefix)
    if(${_prefix} STREQUAL "-l")
      math(EXPR _rest_len "${_length} - 2")
      string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 2 ${_rest_len} PTHREAD_LIB)
    endif()
  endif()
endif()

message(STATUS "pthread=${PTHREAD_LIB}")

ament_export_include_directories(include)

# SOEM defaults to position-dependent code as a static library; ROS shared
# libraries require PIC, so force it here so we can link SOEM into shared libs.
add_compile_options(-fPIC)
remove_definitions(-DWIN32_LEAN_AND_MEAN)

add_subdirectory(SOEM)

ament_export_libraries(soem ${PTHREAD_LIB})
ament_package()