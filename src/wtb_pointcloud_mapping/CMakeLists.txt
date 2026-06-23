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

add_executable(wtb_mapping_node
  src/pointcloud_stitcher.cpp
  src/occupancy_grid_3d.cpp
  src/tower_axis_estimator.cpp
  src/tower_ascent_controller.cpp
  src/wtb_mapping_node.cpp
)

add_executable(lidar_filter
  src/lidar_filter.cpp
)

add_executable(initial_pose_frame_node
  src/initial_pose_frame_node.cpp
)

add_dependencies(wtb_mapping_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(lidar_filter
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(initial_pose_frame_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(wtb_mapping_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(lidar_filter
  ${catkin_LIBRARIES}
)

target_link_libraries(initial_pose_frame_node
  ${catkin_LIBRARIES}
)
