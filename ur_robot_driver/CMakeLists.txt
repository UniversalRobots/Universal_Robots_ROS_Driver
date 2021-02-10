cmake_minimum_required(VERSION 2.8.12)
project(ur_robot_driver)

add_definitions( -DROS_BUILD )

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    control_msgs
    controller_manager
    geometry_msgs
    hardware_interface
    industrial_robot_status_interface
    pluginlib
    roscpp
    sensor_msgs
    std_srvs
    tf
    tf2_geometry_msgs
    tf2_msgs
    trajectory_msgs
    ur_controllers
    ur_dashboard_msgs
    ur_msgs
)
find_package(Boost REQUIRED)
find_package(ur_client_library REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ur_robot_driver_plugin
  CATKIN_DEPENDS
    actionlib
    control_msgs
    controller_manager
    geometry_msgs
    hardware_interface
    pluginlib
    roscpp
    sensor_msgs
    tf
    tf2_geometry_msgs
    tf2_msgs
    trajectory_msgs
    ur_controllers
    ur_dashboard_msgs
    ur_msgs
    std_srvs
  DEPENDS
    Boost
    ur_client_library
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  add_compile_options(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
  add_compile_options(-std=c++0x)
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(ur_robot_driver_plugin
  src/dashboard_client_ros.cpp
  src/hardware_interface.cpp
)
target_link_libraries(ur_robot_driver_plugin ur_client_library::urcl ${catkin_LIBRARIES})
add_dependencies(ur_robot_driver_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ur_robot_driver_node
  src/dashboard_client_ros.cpp
  src/hardware_interface.cpp
  src/hardware_interface_node.cpp
)
target_link_libraries(ur_robot_driver_node ${catkin_LIBRARIES} ur_client_library::urcl)
add_dependencies(ur_robot_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dashboard_client
  src/dashboard_client_ros.cpp
  src/dashboard_client_node.cpp
)
target_link_libraries(dashboard_client ${catkin_LIBRARIES} ur_client_library::urcl)
add_dependencies(dashboard_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robot_state_helper
  src/robot_state_helper.cpp
  src/robot_state_helper_node.cpp
)
target_link_libraries(robot_state_helper ${catkin_LIBRARIES} ur_client_library::urcl)
add_dependencies(robot_state_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest(test/driver.test)
endif()


install(TARGETS ur_robot_driver_plugin ur_robot_driver_node robot_state_helper dashboard_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/tool_communication
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES hardware_interface_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
