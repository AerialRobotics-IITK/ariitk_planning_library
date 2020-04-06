cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detector)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_lib src/obstacle_detector.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${Eigen3_LIBRARIES} ${PCL_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(${PROJECT_NAME}_node src/obstacle_detector_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

##########
# EXPORT #
##########
cs_install()
cs_export()
