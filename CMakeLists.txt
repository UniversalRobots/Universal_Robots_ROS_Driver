cmake_minimum_required(VERSION 2.8.12)
project(ur_modern_driver)

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
    industrial_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf
    trajectory_msgs
    ur_msgs
)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ur_hardware_interface
  CATKIN_DEPENDS
    actionlib
    control_msgs
    controller_manager
    geometry_msgs
    hardware_interface
    industrial_msgs
    roscpp
    sensor_msgs
    trajectory_msgs
    ur_msgs
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

# support indigo's ros_control - This can be removed upon EOL indigo
if("${controller_manager_msgs_VERSION}" VERSION_LESS "0.10.0")
  add_definitions(-DUR_ROS_CONTROL_INTERFACE_OLD_ROS_CONTROL)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Hardware Interface
add_library(ur_hardware_interface
  src/ros/controller.cpp
  src/ros/hardware_interface.cpp)
target_link_libraries(ur_hardware_interface ${catkin_LIBRARIES})


set(${PROJECT_NAME}_SOURCES
    src/ros/action_server.cpp
    src/ros/lowbandwidth_trajectory_follower.cpp
    src/ros/mb_publisher.cpp
    src/ros/rt_publisher.cpp
    src/ros/service_stopper.cpp
    src/ros/trajectory_follower.cpp
    src/ros/urscript_handler.cpp
    src/tcp_socket.cpp
    src/ur/commander.cpp
    src/ur/master_board.cpp
    src/ur/messages.cpp
    src/ur/robot_mode.cpp
    src/ur/rt_state.cpp
    src/ur/server.cpp
    src/ur/stream.cpp)

add_executable(ur_driver ${${PROJECT_NAME}_SOURCES} src/ros_main.cpp)
add_dependencies(ur_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_driver
  ur_hardware_interface
  ${catkin_LIBRARIES}
 )


install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ur_driver ur_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


if (CATKIN_ENABLE_TESTING)
  set(${PROJECT_NAME}_TEST_SOURCES
    tests/ur/master_board.cpp
    tests/ur/robot_mode.cpp
    tests/ur/rt_state.cpp)

  catkin_add_gtest(ur_modern_driver_test ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_TEST_SOURCES} tests/main.cpp)
  target_link_libraries(ur_modern_driver_test ur_hardware_interface ${catkin_LIBRARIES})
endif()
