cmake_minimum_required(VERSION 2.8.3)
project(voxblox_global_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/skeleton_global_planner.cpp
)

cs_add_library(${PROJECT_NAME}_frontier
  src/frontier_evaluator/frontier_evaluator.cpp
)
target_link_libraries(${PROJECT_NAME}_frontier ${Eigen3_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(skeleton_global_planner_node
  src/skeleton_global_planner_node.cpp
)
target_link_libraries(skeleton_global_planner_node ${PROJECT_NAME})

cs_add_executable(frontier_evaluator_node
  src/frontier_evaluator/frontier_evaluator_node.cpp
)
target_link_libraries(frontier_evaluator_node ${PROJECT_NAME}_frontier)

##########
# EXPORT #
##########
cs_install()
cs_export()
