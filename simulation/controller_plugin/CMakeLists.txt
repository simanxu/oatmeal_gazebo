cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(controller_plugin_carwheel SHARED controller_plugin_carwheel.cc)

target_include_directories(controller_plugin_carwheel
  SYSTEM PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)

target_include_directories(controller_plugin_carwheel
  PRIVATE
    ${ROBOTICS_INCLUDE_DIR}
)

target_link_libraries(controller_plugin_carwheel
  PUBLIC
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
    oatmeal_library
)


add_library(controller_plugin_inverted_pendulum SHARED controller_plugin_inverted_pendulum.cc)

target_include_directories(controller_plugin_inverted_pendulum
  SYSTEM PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)

target_include_directories(controller_plugin_inverted_pendulum
  PRIVATE
    ${ROBOTICS_INCLUDE_DIR}
)

target_link_libraries(controller_plugin_inverted_pendulum
  PUBLIC
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
    oatmeal_library
)
