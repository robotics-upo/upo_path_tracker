cmake_minimum_required(VERSION 2.8.3)
project(arco_path_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  upo_actions
)

generate_dynamic_reconfigure_options(
  cfg/nav.cfg
)
#generate_messages(
#  DEPENDENCIES
#)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES displacement
  LIBRARIES securityMargin
  CATKIN_DEPENDS roscpp 
)
###########
## Build ##
###########

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}

)



#add_executable(arco_path_tracker src/navigation.cpp)
#target_link_libraries(arco_path_tracker ${catkin_LIBRARIES} navigator)

#add_executable(arco_path_tracker_2 src/navigation_2.cpp)
#target_link_libraries(arco_path_tracker_2 ${catkin_LIBRARIES} navigator)

add_executable(nav_node src/nav_node.cpp)
add_library(security_margin src/securityMargin.cpp)
add_library(displacement src/displacement.cpp)
target_link_libraries(nav_node ${catkin_LIBRARIES} displacement security_margin )
add_dependencies(nav_node ${PROJECT_NAME}_gencfg)
add_dependencies(nav_node ${catkin_EXPORTED_TARGETS}  )


add_executable(sfm_nav_node src/nav_node_sfm.cpp src/securityMargin.cpp src/sfmcontroller.cpp)
target_link_libraries(sfm_nav_node ${catkin_LIBRARIES})
add_dependencies(sfm_nav_node ${PROJECT_NAME}_gencfg)
add_dependencies(sfm_nav_node ${catkin_EXPORTED_TARGETS})

