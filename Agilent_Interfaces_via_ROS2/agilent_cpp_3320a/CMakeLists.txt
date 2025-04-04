cmake_minimum_required(VERSION 3.8)
project(agilent_cpp_3320a)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Declare the executable
add_executable(agilent_3320a_node src/agilent_3320a_node.cpp)
ament_target_dependencies(agilent_3320a_node rclcpp std_msgs)

add_executable(agilent_3320a_node2 src/agilent_3320a_node2.cpp)
ament_target_dependencies(agilent_3320a_node2 rclcpp std_msgs)

add_executable(agilent_3320a_node2_5 src/agilent_3320a_node2_5.cpp)
ament_target_dependencies(agilent_3320a_node2_5 rclcpp std_msgs)

add_executable(agilent_3320a_node3 src/agilent_3320a_node3.cpp)
ament_target_dependencies(agilent_3320a_node3 rclcpp std_msgs)

add_executable(agilent_3320a_node4 src/agilent_3320a_node4.cpp)
ament_target_dependencies(agilent_3320a_node4 rclcpp std_msgs)

add_executable(agilent_3320a_node5 src/agilent_3320a_node5.cpp)
ament_target_dependencies(agilent_3320a_node5 rclcpp std_msgs)

add_executable(agilent_3320a_node6 src/agilent_3320a_node6.cpp)
ament_target_dependencies(agilent_3320a_node6 rclcpp std_msgs)


install(TARGETS
  agilent_3320a_node
  agilent_3320a_node2
  agilent_3320a_node2_5
  agilent_3320a_node3
  agilent_3320a_node4
  agilent_3320a_node5
  agilent_3320a_node6
  DESTINATION lib/${PROJECT_NAME})


ament_package()
