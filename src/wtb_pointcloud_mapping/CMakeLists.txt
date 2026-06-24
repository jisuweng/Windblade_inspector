cmake_minimum_required(VERSION 3.0.2)
project(wtb_pointcloud_mapping)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  livox_laser_simulation
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
    std_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_eigen
    pcl_ros
    pcl_conversions
    livox_laser_simulation
  DEPENDS PCL EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(wtb_mapping_core
  src/mapping_config.cpp
  src/mapping_pipeline.cpp
  src/pointcloud_stitcher.cpp
  src/occupancy_grid_3d.cpp
  src/ros_conversions.cpp
)

add_executable(wtb_mapping_node
  src/wtb_mapping_node.cpp
)

add_executable(initial_pose_frame_node
  src/initial_pose_frame_node.cpp
)

add_executable(wtb_tower_detector_node
  src/wtb_tower_detector_node.cpp
)

add_dependencies(wtb_mapping_core
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(wtb_mapping_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(initial_pose_frame_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(wtb_tower_detector_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(wtb_mapping_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(wtb_mapping_node
  wtb_mapping_core
  ${catkin_LIBRARIES}
)

target_link_libraries(initial_pose_frame_node
  ${catkin_LIBRARIES}
)

target_link_libraries(wtb_tower_detector_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
