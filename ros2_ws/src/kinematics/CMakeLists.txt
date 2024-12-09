cmake_minimum_required(VERSION 3.8)
project(kinematics)

# C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Define executables
add_executable(forward_kinematics_node
    src/forward_kinematics_node.cpp 
    src/Hexapod.cpp 
    src/SpiderLeg.cpp
)
add_executable(inverse_kinematics_node
    src/inverse_kinematics_node.cpp 
    src/Hexapod.cpp 
    src/SpiderLeg.cpp
)
add_executable(kinematics_test
    src/main.cpp 
    src/Hexapod.cpp 
    src/SpiderLeg.cpp
)

# Link libraries and include directories
target_include_directories(forward_kinematics_node PUBLIC include)
target_include_directories(inverse_kinematics_node PUBLIC include)
target_include_directories(kinematics_test PUBLIC include)

ament_target_dependencies(forward_kinematics_node rclcpp std_msgs)
ament_target_dependencies(inverse_kinematics_node rclcpp std_msgs)
ament_target_dependencies(kinematics_test rclcpp std_msgs)

# Install executables
install(TARGETS 
    forward_kinematics_node
    inverse_kinematics_node
    kinematics_test
    DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
    DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
