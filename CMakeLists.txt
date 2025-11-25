cmake_minimum_required(VERSION 3.0.2)
project(neat_movement)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  move_base_msgs
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  tf2_ros
  tf2_geometry_msgs
)

# Generate actions
add_action_files(
  DIRECTORY action
  FILES SimpleMove.action
)

# Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/SimpleMoveParams.cfg
)

# Generate messages
generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs move_base_msgs actionlib actionlib_msgs dynamic_reconfigure tf2_ros tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Simple move server node
add_executable(simple_move_server src/simple_move_server.cpp)
add_dependencies(simple_move_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_move_server ${catkin_LIBRARIES})
