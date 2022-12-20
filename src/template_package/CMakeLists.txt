cmake_minimum_required(VERSION 3.5)
project(template_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# This is needed for in-node bag reading:
find_package(rosbag2_cpp REQUIRED)

#Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TemplateMsg.msg"
  DEPENDENCIES std_msgs
 )

# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)

include_directories(
  include
  # ${OpenCV_INCLUDE_DIRS}
)

## Executables
add_executable(template_node src/ros_node.cpp src/agnostic_logic_class.cpp)
ament_target_dependencies(template_node rclcpp std_msgs)
# target_link_libraries(template_node pointcloud_utils::pointcloud_utils)
# rosidl_target_interfaces(template_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Other examples
# # In-node bag reading:
add_executable(bagread_node src/other_examples/bag_read_node.cpp)
ament_target_dependencies(bagread_node rclcpp std_msgs sensor_msgs rosbag2_cpp)

# # Multithreaded Callbacks/Node Execution:
add_executable(multithreaded_node src/other_examples/multithreaded_node.cpp src/agnostic_logic_class.cpp)
ament_target_dependencies(multithreaded_node rclcpp std_msgs)


install(
  TARGETS 
    template_node
    bagread_node
    multithreaded_node
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
