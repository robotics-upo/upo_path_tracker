cmake_minimum_required(VERSION 2.8.3)
project(upo_path_tracker)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  upo_actions
  dynamic_reconfigure
  theta_star_2d
)

find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/Doxyfile)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( ${PROJECT_NAME}_doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

generate_dynamic_reconfigure_options(
  cfg/ArcoPathTracker.cfg
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

#Standard path tracker node used currently in the NIx project. This one is based in the node used in arco project.
add_executable(simple_path_tracker_node src/simple_path_tracker_node.cpp 
                                        src/simple_path_tracker/UpoNavigationSimplePathTracker.cpp
                                        src/utils/geometry.cpp
                                        src/utils/transforms.cpp)

target_link_libraries(simple_path_tracker_node ${catkin_LIBRARIES} ${PROJECT_INCLUDE_DIRS})
add_dependencies(simple_path_tracker_node ${PROJECT_NAME}_gencfg  
                                          ${catkin_EXPORTED_TARGETS}  
                                          ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Alternative Path tracker node used in the arco project based in the social force model

add_executable(sfm_path_tracker src/sfm_path_tracker.cpp 
                                src/sfm_path_tracker/SFMController.cpp 
                                src/arco_path_tracker/SecurityMargin.cpp)

target_link_libraries(sfm_path_tracker ${catkin_LIBRARIES})
add_dependencies(sfm_path_tracker ${PROJECT_NAME}_gencfg 
                                  ${catkin_EXPORTED_TARGETS}  
                                  ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Path tracker node used in the arco project
add_executable(arco_path_tracker_node src/arco_path_tracker_node.cpp
                                      src/arco_path_tracker/ArcoPathTracker.cpp 
                                      src/arco_path_tracker/SecurityMargin.cpp)

target_link_libraries(arco_path_tracker_node ${catkin_LIBRARIES})
add_dependencies(arco_path_tracker_node ${PROJECT_NAME}_gencfg  
                                   ${catkin_EXPORTED_TARGETS}  
                                   ${${PROJECT_NAME}_EXPORTED_TARGETS})
