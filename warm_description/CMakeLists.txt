cmake_minimum_required(VERSION 2.8.3)

project(warm_description)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(joint_publisher src/joint_publisher.cpp)
target_link_libraries(joint_publisher ${catkin_LIBRARIES})

