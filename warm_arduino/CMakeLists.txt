cmake_minimum_required(VERSION 2.8.3)
project(warm_arduino)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

## Declare ROS messages and services
add_message_files(FILES ArmJointState.msg)

generate_messages(DEPENDENCIES std_msgs)
#catkin_package()


catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(JointAngle2Steps scripts/JointAngle2Steps.cpp)
target_link_libraries(JointAngle2Steps ${catkin_LIBRARIES})
