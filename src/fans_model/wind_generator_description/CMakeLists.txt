cmake_minimum_required(VERSION 2.8.3)

project(wind_generator_description)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(wind_generator_pose_lock_plugin
	src/wind_generator_pose_lock_plugin.cpp
)
target_link_libraries(wind_generator_pose_lock_plugin
	${GAZEBO_LIBRARIES}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(TARGETS wind_generator_pose_lock_plugin
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
