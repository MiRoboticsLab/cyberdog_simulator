cmake_minimum_required(VERSION 3.5)
project(cyberdog_visual)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(lcm REQUIRED)
find_package(urdf REQUIRED)
set(dependencies
  rclcpp
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  gazebo_msgs
  urdf
  lcm
  Eigen3
)

include_directories(
  include
  lcm_type
)

add_executable(cyberdog_visual
  src/cyberdog_visual_node.cpp
  src/cyberdog_visual.cpp
)
ament_target_dependencies(cyberdog_visual ${dependencies})
target_link_libraries(cyberdog_visual lcm)

install(TARGETS cyberdog_visual
    EXPORT export_${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()