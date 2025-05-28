cmake_minimum_required(VERSION 3.8)
project(ros2_robot_ur5e)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ur_robot_driver REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

install(DIRECTORY 
  config
  description
  launch
  meshes
  rviz
  scripts
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES 
  .setup_assistant 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
