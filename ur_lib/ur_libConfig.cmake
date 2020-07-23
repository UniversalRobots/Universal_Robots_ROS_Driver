include(CMakeFindDependencyMacro)

if(NOT TARGET ur_lib::ur_lib)
  include("${CMAKE_CURRENT_LIST_DIR}/ur_libTargets.cmake")
endif()

# This is for catkin compatibility. Better use target_link_libraries(<my_target> ur_lib::ur_lib)
set(ur_lib_LIBRARIES ur_lib::ur_lib)
get_target_property(ur_lib_INCLUDE_DIRS ur_lib::ur_lib INTERFACE_INCLUDE_DIRECTORIES)

