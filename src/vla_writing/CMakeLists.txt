cmake_minimum_required(VERSION 3.0.2)
project(vla_writing)

add_compile_options(-std=c++11 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs std_srvs geometry_msgs trajectory_msgs
  tf tf2_ros tf2_geometry_msgs moveit_commander moveit_msgs
  sensor_msgs image_transport cv_bridge gazebo_ros gazebo_msgs
  controller_manager controller_manager_msgs visualization_msgs message_generation
)
find_package(gazebo REQUIRED)

# Install/link the ROS-independent Python package alongside generated service
# modules.  Without this call catkin creates only ``vla_writing.srv`` and the
# parser modules are invisible after ``source devel/setup.bash``.
catkin_python_setup()

add_service_files(FILES WriteText.srv DrawImage.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ink_visual_plugin
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs trajectory_msgs tf tf2_ros
    tf2_geometry_msgs moveit_commander moveit_msgs sensor_msgs image_transport
    cv_bridge gazebo_ros gazebo_msgs controller_manager message_runtime
    controller_manager_msgs visualization_msgs
  DEPENDS gazebo
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(ink_visual_plugin SHARED src/ink_visual_plugin.cpp)
add_dependencies(ink_visual_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ink_visual_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/writing_node.py scripts/pen_monitor.py
  scripts/paper_view.py scripts/write_cli.py scripts/scene_initializer.py
  scripts/scene_markers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ink_visual_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch config urdf worlds models data rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY vla_writing/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/vla_writing)
install(FILES README.md setup.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
