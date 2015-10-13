cmake_minimum_required(VERSION 2.8.3)
project(sample_move_arm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  rosbag

  pluginlib
  cmake_modules
  geometry_msgs
  ar_track_alvar
  visualization_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    rosbag
    roscpp
    geometry_msgs
    ar_track_alvar
    visualization_msgs
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(move_arm src/move_arm.cpp)
target_link_libraries(move_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_arm DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/plot_poses.py scripts/transformed_waypoints.py scripts/plot_transformed.py 
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})