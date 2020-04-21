cmake_minimum_required(VERSION 2.8.3)
project(upo_path_tracker)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  upo_actions
  dynamic_reconfigure
  theta_star_2d
)
generate_dynamic_reconfigure_options(
  cfg/nav.cfg
  cfg/SimplePathTracker.cfg
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES displacement
  LIBRARIES SimplePathTracker
  LIBRARIES securityMargin
  CATKIN_DEPENDS roscpp upo_actions theta_star_2d
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_path_tracker_node src/simple_path_tracker_node.cpp src/simple_path_tracker/UpoNavigationSimplePathTracker.cpp)
target_link_libraries(simple_path_tracker_node ${catkin_LIBRARIES})
add_dependencies(simple_path_tracker_node ${PROJECT_NAME}_gencfg  ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(test_node src/test.cpp)
#target_link_libraries(test_node ${catkin_LIBRARIES}  )
#add_dependencies(test_node ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(upo_path_tracker src/navigation.cpp)
#target_link_libraries(upo_path_tracker ${catkin_LIBRARIES} navigator)

#add_executable(upo_path_tracker_2 src/navigation_2.cpp)
#target_link_libraries(upo_path_tracker_2 ${catkin_LIBRARIES} navigator)

#add_executable(nav_node src/nav_node.cpp)
#add_library(security_margin src/securityMargin.cpp)
#add_library(displacement src/displacement.cpp )
#add_dependencies(displacement ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(nav_node ${catkin_LIBRARIES} displacement security_margin )
#add_dependencies(nav_node ${PROJECT_NAME}_gencfg  ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})
#
#add_executable(sfm_nav_node src/nav_node_sfm.cpp src/securityMargin.cpp src/sfmcontroller.cpp)
#target_link_libraries(sfm_nav_node ${catkin_LIBRARIES})
#add_dependencies(sfm_nav_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS}  ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_library(security_margin src/securityMargin.cpp)
#add_library(PathTracker src/PathTracker.cpp )

