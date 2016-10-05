cmake_minimum_required(VERSION 2.8.3)
project(p4)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED
  roscpp
)


catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# cs_add_executable(minimal_joint_controller_p4 src/minimal_joint_controller.cpp)
# cs_install()
# cs_export()

