cmake_minimum_required(VERSION 3.5)
project(cyberdog_gazebo)

set(CMAKE_IGNORE_PATH "/usr/local")

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

if(ONBOARD_BUILD)
  SET (THIS_COM "../" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
else(ONBOARD_BUILD)
  SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
endif(ONBOARD_BUILD)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(cyberdog_msg REQUIRED)
find_package(lcm REQUIRED)


set(dependencies
  rclcpp
  Eigen3
  gazebo_msgs
  gazebo_dev
  gazebo_ros
  cyberdog_msg
)

file(GLOB_RECURSE sources "src/control_parameters/*.cpp"
                          "src/sim_utilities/*.cpp"
                          "src/utilities/*.cpp")

include_directories(include)
include_directories(include/ctrl_ros)
include_directories("third-party/ParamHandler")
include_directories(${CMAKE_BINARY_DIR})

add_subdirectory("third-party/ParamHandler")

add_library(foot_contact_plugin SHARED src/foot_contact_plugin.cpp)
ament_target_dependencies(foot_contact_plugin ${dependencies})
target_link_libraries(foot_contact_plugin ${GAZEBO_LIBRARIES} lcm)

add_library(legged_plugin SHARED ${sources} src/legged_plugin.cpp src/legged_simparam.cpp src/lcmhandler.cpp)
ament_target_dependencies(legged_plugin ${dependencies})
target_link_libraries(legged_plugin ${GAZEBO_LIBRARIES} param_handler pthread lcm)

install(TARGETS legged_plugin param_handler foot_contact_plugin
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Mark other files for installation
install(
  DIRECTORY
  launch
  world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
