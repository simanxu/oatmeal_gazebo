project(oatmeal_lib CXX)

cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 17)

add_library(oatmeal_controller SHARED
  src/oatmeal_controller.cc
)

target_include_directories(oatmeal_controller
PUBLIC

)

target_link_libraries(oatmeal_controller
PUBLIC
  eigen
  rbdl
  qpOASES
  PSOPT
)

# add_subdirectory(test)
