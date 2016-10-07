cmake_minimum_required(VERSION 2.8.3)
project(p4)
find_package(catkin_simple REQUIRED)

catkin_simple()


cs_add_executable(joint_controller_p4 src/joint_controller.cpp)
cs_add_executable(sin_commander_p4 src/sin_commander.cpp)
cs_install()
cs_export()

