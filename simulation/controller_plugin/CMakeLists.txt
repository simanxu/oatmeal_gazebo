cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/carwheel/src)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(controller_plugin_c SHARED controller_plugin.cc)

target_link_libraries(controller_plugin_c
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
)
