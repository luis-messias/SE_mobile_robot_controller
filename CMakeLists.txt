cmake_minimum_required(VERSION 3.8)
project(SE_mobile_robot_controller)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
