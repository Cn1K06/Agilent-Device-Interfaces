cmake_minimum_required(VERSION 3.8)
project(agilent_3485a_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(agilent_3485a_node src/agilent_3485a_node.cpp)
ament_target_dependencies(agilent_3485a_node rclcpp std_msgs)

add_executable(agilent_3485a_node2 src/agilent_3485a_node2.cpp)
ament_target_dependencies(agilent_3485a_node2 rclcpp std_msgs)

add_executable(agilent_3485a_node3 src/agilent_3485a_node3.cpp)
ament_target_dependencies(agilent_3485a_node3 rclcpp std_msgs)
target_link_libraries(agilent_3485a_node3 ${Boost_LIBRARIES})

add_executable(agilent_3485a_node4 src/agilent_3485a_node4.cpp)
ament_target_dependencies(agilent_3485a_node4 rclcpp std_msgs)


add_executable(agilent_3485a_node5 src/agilent_3485a_node5.cpp)
ament_target_dependencies(agilent_3485a_node5 rclcpp std_msgs)

add_executable(agilent_3485a_node6 src/agilent_3485a_node6.cpp)
ament_target_dependencies(agilent_3485a_node6 rclcpp std_msgs)

add_executable(agilent_3485a_node7 src/agilent_3485a_node7.cpp)
ament_target_dependencies(agilent_3485a_node7 rclcpp std_msgs)



install(TARGETS
  agilent_3485a_node
  agilent_3485a_node2
  agilent_3485a_node3
  agilent_3485a_node4
  agilent_3485a_node5
  agilent_3485a_node6
  agilent_3485a_node7
  DESTINATION lib/${PROJECT_NAME})

ament_package()
