add_library(eigen INTERFACE)

# Avoids using LGPL code.
target_compile_definitions(eigen INTERFACE EIGEN_MPL2_ONLY)

# Hides local Eigen symbols to avoid conflict when loading third-party libraries also built with Eigen.
target_link_options(eigen INTERFACE
  "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/hide_eigen.ld"
)
set_target_properties(eigen PROPERTIES
  INTERFACE_LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/hide_eigen.ld)

# Compatibility library for targets that require relative include path of Eigen, i.e.
# #include "Eigen/Dense" or #include <Eigen/Dense>.
add_library(eigen_compat INTERFACE)
target_include_directories(eigen_compat INTERFACE
  ${XLEOPARD_INCLUDE_DIRS}/third_party/eigen
)
target_compile_definitions(eigen_compat INTERFACE EIGEN_MPL2_ONLY)
target_link_options(eigen_compat INTERFACE
  "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/hide_eigen.ld"
)
set_target_properties(eigen_compat PROPERTIES
  INTERFACE_LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/hide_eigen.ld)
