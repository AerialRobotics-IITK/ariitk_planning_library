cmake_minimum_required(VERSION 2.8.3)
project(voxblox_global_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/skeleton_global_planner.cpp
)

############
# BINARIES #
############
cs_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
