cmake_minimum_required(VERSION 3.8)
project(cyberdog_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cyberdog_msg REQUIRED)
find_package(lcm REQUIRED)

include_directories(include)

add_executable(cyberdogmsg_sender src/cyberdogmsg_sender.cpp)
ament_target_dependencies(cyberdogmsg_sender rclcpp rclcpp_action cyberdog_msg)

add_executable(keybroad_commander src/keybroad_commander.cpp)
ament_target_dependencies(keybroad_commander rclcpp rclcpp_action cyberdog_msg)
target_link_libraries(keybroad_commander lcm) 

ament_package()
