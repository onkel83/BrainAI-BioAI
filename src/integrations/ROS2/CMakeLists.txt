cmake_minimum_required(VERSION 3.8)
project(bioai_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 1. Header-Verzeichnisse angeben (dort liegt BioAI.hpp und Co.)
include_directories(include)

# 2. Die vorhandene Binärdatei als Library bekannt machen
add_library(bioai_lib SHARED IMPORTED)
set_target_properties(bioai_lib PROPERTIES 
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/BioAI_ULTRA.dll"
)

# 3. Den ROS-Knoten bauen
add_executable(bioai_bridge_node src/bioai_ros2_node.cpp)
target_link_libraries(bioai_bridge_node
    rclcpp::rclcpp
    std_msgs::std_msgs
    bioai_lib
)

ament_package()