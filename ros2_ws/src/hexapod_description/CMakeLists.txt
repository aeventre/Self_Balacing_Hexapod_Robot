cmake_minimum_required(VERSION 3.8)
project(hexapod_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install URDF and Xacro files
install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
)

# Install RViz configuration files (if any)
install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
