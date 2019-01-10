cmake_minimum_required(VERSION 2.8.3)
project(taros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf
    moveit_msgs
    moveit_visual_tools
    )

catkin_package(
    INCLUDE_DIRS #include
    LIBRARIES taros
    CATKIN_DEPENDS rospy roscpp moveit_core moveit_ros_planning moveit_ros_planning_interface tf moveit_msgs moveit_visual_tools
    #DEPENDS system_lib
    )

include_directories(
    #include
    ${catkin_INCLUDE_DIRS}
    )

