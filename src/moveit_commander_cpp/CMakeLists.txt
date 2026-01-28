cmake_minimum_required(VERSION 3.8)
project(moveit_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_interfaces REQUIRED)

add_executable(moveit_planning_exe src/moveit_plan_publisher.cpp)
ament_target_dependencies(moveit_planning_exe rclcpp moveit_ros_planning_interface)

install(TARGETS
  moveit_planning_exe
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
