cmake_minimum_required(VERSION 3.5)
project(static_balancer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(static_balancer_node src/static_balancer.cpp)
ament_target_dependencies(static_balancer_node rclcpp sensor_msgs std_msgs)

install(TARGETS
  static_balancer_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

