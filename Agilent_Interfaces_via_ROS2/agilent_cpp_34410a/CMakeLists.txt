cmake_minimum_required(VERSION 3.8)
project(agilent_34410a_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(agilent_34410a_node src/agilent_34410a_node.cpp)
ament_target_dependencies(agilent_34410a_node rclcpp std_msgs)

add_executable(agilent_34410a_node2 src/agilent_34410a_node2.cpp)
ament_target_dependencies(agilent_34410a_node2 rclcpp std_msgs)

add_executable(agilent_34410a_node3 src/agilent_34410a_node3.cpp)
ament_target_dependencies(agilent_34410a_node3 rclcpp std_msgs)

add_executable(agilent_34410a_node4 src/agilent_34410a_node4.cpp)
ament_target_dependencies(agilent_34410a_node4 rclcpp std_msgs)

add_executable(agilent_34410a_node2_5 src/agilent_34410a_node2_5.cpp)
ament_target_dependencies(agilent_34410a_node2_5 rclcpp std_msgs)

add_executable(agilent_34410a_node5 src/agilent_34410a_node5.cpp)
ament_target_dependencies(agilent_34410a_node5 rclcpp std_msgs)

add_executable(agilent_34410a_node6 src/agilent_34410a_node6.cpp)
ament_target_dependencies(agilent_34410a_node6 rclcpp std_msgs)


install(TARGETS
  agilent_34410a_node
  agilent_34410a_node2
  agilent_34410a_node3
  agilent_34410a_node4
  agilent_34410a_node2_5
  agilent_34410a_node5
  agilent_34410a_node6
  DESTINATION lib/${PROJECT_NAME})


ament_package()
