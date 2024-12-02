cmake_minimum_required(VERSION 3.8)
project(support_polygon_calculator)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hexapod_msgs REQUIRED)  # Custom messages for your project

# Add the executable
add_executable(support_polygon_calculator src/support_polygon_calculator.cpp)

# Link dependencies
ament_target_dependencies(support_polygon_calculator
  rclcpp
  geometry_msgs
  std_msgs
  hexapod_msgs
)

# Include directories
target_include_directories(support_polygon_calculator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS support_polygon_calculator
  DESTINATION lib/${PROJECT_NAME}
)

# Install include headers
install(DIRECTORY include/
  DESTINATION include/
)

# Testing (if required)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package
ament_package()
