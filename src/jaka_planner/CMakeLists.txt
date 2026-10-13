cmake_minimum_required(VERSION 3.8)
project(jaka_planner)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Declare executable targets
add_executable(moveit_server src/moveit_server.cpp)
add_executable(recover_robot src/recover_robot.cpp)

# Link libraries
ament_target_dependencies(moveit_server
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sensor_msgs
  action_msgs
  control_msgs
  trajectory_msgs
)

# For including directories in ROS 2
target_include_directories(moveit_server
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

target_include_directories(recover_robot
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

# Specify where to find external libraries
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Add the directory to the search path
link_directories(${LIBRARY_DIR})

# Find the library and link it properly
target_link_libraries(moveit_server ${LIBRARY_DIR}/libjakaAPI.so)
target_link_libraries(recover_robot ${LIBRARY_DIR}/libjakaAPI.so)

# Install executables
install(TARGETS
  moveit_server
  recover_robot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()



