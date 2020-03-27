cmake_minimum_required(VERSION 2.8.3)
project(global_planner)

find_package(catkin_simple)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

cs_add_library(map_server src/map_server.cpp)

cs_add_executable(map_server_node src/map_server_node.cpp)
target_link_libraries(map_server_node map_server)

cs_add_library(a_star_planner src/a_star_planner.cpp src/a_star.cpp)
target_link_libraries(a_star_planner map_server ${Eigen3_LIBRARIES})

cs_add_executable(a_star_planner_node src/a_star_planner_node.cpp)
target_link_libraries(a_star_planner_node a_star_planner)

cs_install()
cs_export()
