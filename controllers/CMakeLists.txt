project(oatmeal_library CXX)

cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 17)

add_library(oatmeal_library SHARED
  oatmeal_dynamic.cc
  oatmeal_planner.cc
  oatmeal_controller.cc
)

target_include_directories(oatmeal_library
PUBLIC

)

target_link_libraries(oatmeal_library
PUBLIC
  eigen
  rbdl
  qpOASES
  PSOPT
)

# add_subdirectory(test)
