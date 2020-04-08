cmake_minimum_required(VERSION 2.8.3)
project(frontier_explorer)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/frontier_evaluator.cpp
  src/frontier_planner.cpp
)
target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(frontier_planner_node
  src/frontier_planner_node.cpp
)
target_link_libraries(frontier_planner_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
