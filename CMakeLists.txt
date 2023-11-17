cmake_minimum_required(VERSION 2.8.3)
project(upo_path_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug) #None, Debug, Release, RelWithDebInfo, MinSizeRel

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  upo_actions
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/nav.cfg
  cfg/PathTracker.cfg
)
#generate_messages(
#  DEPENDENCIES
#)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES displacement
  LIBRARIES PathTracker
  LIBRARIES securityMargin
  CATKIN_DEPENDS roscpp upo_actions
)
###########
## Build ##
###########

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)


add_executable(nav_node_2 src/nav_node_2.cpp src/PathTracker.cpp src/securityMargin.cpp)
target_link_libraries(nav_node_2  ${catkin_LIBRARIES})
add_dependencies(nav_node_2 ${PROJECT_NAME}_gencfg  ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(new_arco_traj_tracker_node src/new_arco_traj_tracker_node.cpp src/new_arco_traj_tracker.cpp)
target_link_libraries(new_arco_traj_tracker_node ${catkin_LIBRARIES})
add_dependencies(new_arco_traj_tracker_node ${PROJECT_NAME}_gencfg  ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})
