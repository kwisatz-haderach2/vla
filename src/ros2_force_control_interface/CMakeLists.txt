cmake_minimum_required(VERSION 3.8)
project(ros2_force_control_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(service_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ContactState.msg"
  "msg/ForceCommand.msg"
  "msg/ForceState.msg"
  "msg/ForceTrajectory.msg"
  "srv/SetCompliance.srv"
  "srv/SetForceControl.srv"
  "action/ExecuteForceWriting.action"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs service_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  src/force_control_gateway.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
