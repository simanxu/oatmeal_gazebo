add_library(rbdl STATIC
  src/Constraint_Contact.cc
  src/Constraint_Loop.cc
  src/Constraints.cc
  src/Dynamics.cc
  src/Joint.cc
  src/Kinematics.cc
  src/Logging.cc
  src/Model.cc
  src/rbdl_errors.cc
  src/rbdl_mathutils.cc
  src/rbdl_utils.cc
  src/rbdl_version.cc
)
target_link_libraries(rbdl
  PUBLIC
    eigen_compat
)
target_include_directories(rbdl
  SYSTEM PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_include_directories(rbdl
  PUBLIC
    ${XLEOPARD_INCLUDE_DIRS}
)
set_target_properties(rbdl PROPERTIES POSITION_INDEPENDENT_CODE ON)
