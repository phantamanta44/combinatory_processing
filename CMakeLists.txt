cmake_minimum_required(VERSION 2.8.3)
project(combinatory_processing)

find_package(catkin REQUIRED COMPONENTS rospy)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    LIBRARIES combinatory_processing
    CATKIN_DEPENDS rospy
)

catkin_python_setup()