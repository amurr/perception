cmake_minimum_required(VERSION 3.8)
project(moveit_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- interfaces (msg + action) ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SubMovement.msg"
  "action/ExecuteSequence.action"
  DEPENDENCIES geometry_msgs
)

# --- executable ---
# rosidl_generate_interfaces(${PROJECT_NAME}) above already creates a custom
# target called "moveit_master", so the executable needs a distinct target
# name. OUTPUT_NAME keeps the installed binary (and `ros2 run`) as before.
add_executable(moveit_master_node src/moveit_master.cpp)
set_target_properties(moveit_master_node PROPERTIES OUTPUT_NAME moveit_master)
ament_target_dependencies(moveit_master_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
)

# use the interfaces generated in this same package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(moveit_master_node "${cpp_typesupport_target}")

install(TARGETS moveit_master_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()