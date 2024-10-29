cmake_minimum_required(VERSION 3.8)
project(smrr_executions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(smrr_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)   # include header files


add_executable(pose_to_angle_node  src/pose_to_angle.cpp)
ament_target_dependencies(pose_to_angle_node     rclcpp geometry_msgs moveit_ros_planning_interface yaml-cpp)
include_directories(${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(${YAML_CPP_LIBRARIES})
target_link_libraries(pose_to_angle_node ${YAML_CPP_LIBRARIES})

add_executable(move_pose  src/move_pose.cpp)
ament_target_dependencies(move_pose     rclcpp geometry_msgs moveit_ros_planning_interface )



add_library(arm_control_server SHARED src/arm_control_server.cpp)
target_include_directories(arm_control_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(arm_control_server
  PRIVATE "TASK_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(arm_control_server
  "smrr_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface")
rclcpp_components_register_node(arm_control_server 
  PLUGIN "smrr_executions::ArmControlServer" 
  EXECUTABLE arm_control_server_node
)

add_library(arm_control_client SHARED src/arm_control_client.cpp)
target_include_directories(arm_control_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(arm_control_client
  PRIVATE "TASK_CLIENT_CPP_BUILDING_DLL")
ament_target_dependencies(arm_control_client
  "smrr_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
  "yaml-cpp")
target_link_libraries(arm_control_client ${YAML_CPP_LIBRARIES})
rclcpp_components_register_node(arm_control_client 
  PLUGIN "smrr_executions::ArmControlClient" 
  EXECUTABLE arm_control_client_node
)



INSTALL(TARGETS
pose_to_angle_node
move_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  arm_control_server
  arm_control_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()
