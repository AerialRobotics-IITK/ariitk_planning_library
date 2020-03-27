cmake_minimum_required(VERSION 2.8.3)
project(global_planner)

find_package(catkin_simple)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(map_server src/map_server.cpp)

cs_add_executable(map_server_node src/map_server_node.cpp)
target_link_libraries(map_server_node map_server)

cs_install()
cs_export()
