cmake_minimum_required(VERSION 3.8)
project(mav_msgs)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Actuators.msg
  msg/AttitudeThrust.msg
  msg/FilteredSensorData.msg
  msg/GpsWaypoint.msg
  msg/RateThrust.msg
  msg/RollPitchYawrateThrust.msg
  msg/Status.msg
  msg/TorqueThrust.msg
  DEPENDENCIES std_msgs geometry_msgs nav_msgs trajectory_msgs
)

# for making the header files available in 
# ${mav_msgs_INCLUDE_DIRS} path so that other packages can access it.
ament_export_include_directories(include) 
ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
