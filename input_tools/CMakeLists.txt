find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ros_joy_keyboard
  ros_joy_keyboard.cc
)
target_include_directories(ros_joy_keyboard
  SYSTEM PRIVATE
    ${roscpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)
target_link_libraries(ros_joy_keyboard
  PRIVATE
    ${roscpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
)
