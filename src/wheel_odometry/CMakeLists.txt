cmake_minimum_required(VERSION 3.0.2)
project(wheel_odometry)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_publisher
  robot_state_publisher
  rospy
  rviz
)

LINK_DIRECTORIES(/usr/local/lib)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES anrol
#  CATKIN_DEPENDS controller_manager gazebo_ros joint_state_publisher robot_state_publisher rospy rviz
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(wheel_node src/odom.cpp)
target_link_libraries(wheel_node ${catkin_LIBRARIES})

add_executable(primitive_wheel_node src/primitive_odom.cpp)
target_link_libraries(primitive_wheel_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  nodes/odom_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


