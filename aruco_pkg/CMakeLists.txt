cmake_minimum_required(VERSION 2.8.3)
project(aruco_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp std_msgs image_transport cv_bridge tf pcl_ros
)

set(aruco_DIR /usr/local/share/aruco)
set(marker_mapper_DIR /usr/local/lib/cmake)
FIND_PACKAGE(aruco REQUIRED )
find_package(OpenCV REQUIRED )
find_package(marker_mapper REQUIRED)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES aruco_pkg
#  CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge tf pcl_ros
#  DEPENDS system_lib
)


include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${marker_mapper_INCLUDE_DIRS}
)


add_executable(aruco_single_estimator
  src/aruco_single_estimator.cpp
)
add_dependencies(aruco_single_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_single_estimator
${catkin_LIBRARIES}
${aruco_LIBS}
${OpenCV_LIBS}
)


add_executable(aruco_map_estimator
  src/aruco_map_estimator.cpp
)
add_dependencies(aruco_map_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_map_estimator
${catkin_LIBRARIES}
${aruco_LIBS}
${OpenCV_LIBS}
)


add_executable(marker_mapper_node
  src/marker_mapper_node.cpp
)
add_dependencies(marker_mapper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_mapper_node
${catkin_LIBRARIES}
${aruco_LIBS}
${OpenCV_LIBS}
${marker_mapper_LIBS}
)


add_executable(create_path src/create_path.cpp)
add_dependencies(create_path ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_path ${catkin_LIBRARIES})


add_executable(path_tracker_node src/path_tracker.cpp)
add_dependencies(path_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_tracker_node ${catkin_LIBRARIES})


add_executable(pose_grapher_node 
src/pose_grapher.cpp
include/CameraPoseVisualization.cpp)
add_dependencies(pose_grapher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_grapher_node ${catkin_LIBRARIES})

add_executable(filter_node src/fusion_filter.cpp)
add_dependencies(filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_node ${catkin_LIBRARIES})


