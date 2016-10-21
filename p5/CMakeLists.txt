cmake_minimum_required(VERSION 2.8.3)
project(p5)
find_package(catkin_simple REQUIRED)

catkin_simple()


cs_add_executable(sin_commander_p5 src/sin_commander.cpp)
cs_install()
cs_export()

