cmake_minimum_required(VERSION 3.0.2)
project(livox_to_pointcloud2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
  livox_laser_simulation
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS livox_laser_simulation roscpp sensor_msgs
)

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(livox_to_pointcloud2_node
  src/livox_to_pointcloud2_ros1.cpp
)
target_include_directories(livox_to_pointcloud2_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(livox_to_pointcloud2_node
  ${catkin_LIBRARIES}
)
add_dependencies(livox_to_pointcloud2_node
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS livox_to_pointcloud2_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_livox_converter test/test_livox_converter.cpp)
  if(TARGET test_livox_converter)
    target_include_directories(test_livox_converter PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_livox_converter ${catkin_LIBRARIES})
  endif()
endif()
