cmake_minimum_required(VERSION 3.5)
project(rosbag2_api_examples)

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

#This is needed for in-node bag reading:
find_package(rosbag2_cpp REQUIRED)

#Generate custom messages
# rosidl_generate_interfaces(${PROJECT_NAME}
#   DEPENDENCIES std_msgs
#  )

# find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package (Boost COMPONENTS filesystem)

include_directories(
  include
   ${Boost_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
)

## Executables
# In-node bag reading:
add_executable(bagread_node src/bag_read_node.cpp)
ament_target_dependencies(bagread_node rclcpp std_msgs sensor_msgs rosbag2_cpp)
target_link_libraries(bagread_node ${Boost_FILESYSTEM_LIBRARY})

install(
  TARGETS 
    bagread_node
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
