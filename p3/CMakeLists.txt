cmake_minimum_required(VERSION 2.8.3)
project(p3)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Executables
cs_add_executable(minimal_simulator_p3 src/minimal_simulator.cpp)
cs_add_executable(minimal_controller_p3 src/minimal_controller.cpp)
cs_add_executable(sin_client_p3 src/sin_client.cpp)
cs_add_executable(sin_commander_p3 src/sin_commander.cpp)
cs_install()
cs_export()
    
