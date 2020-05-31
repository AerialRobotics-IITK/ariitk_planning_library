cmake_minimum_required(VERSION 2.8.3)
project(rviz_visualizer)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/visualizer.cpp
)

############
# BINARIES #
############

##########
# EXPORT #
##########
cs_install()
cs_export()
