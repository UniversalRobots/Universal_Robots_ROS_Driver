cmake_minimum_required(VERSION 2.8.12)
project(ur_rtde_driver)

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
    eigen_conversions
    geometry_msgs
    hardware_interface
    kdl_parser
    industrial_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf
    tf_conversions
    trajectory_msgs
    ur_msgs
)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ur_rtde_driver
  CATKIN_DEPENDS
    actionlib
    control_msgs
    controller_manager
    eigen_conversions
    geometry_msgs
    hardware_interface
    kdl_parser
    industrial_msgs
    roscpp
    sensor_msgs
    trajectory_msgs
    ur_msgs
    std_srvs
    tf
    tf_conversions
  DEPENDS
    Boost
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
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ur_rtde_driver
    src/comm/tcp_socket.cpp
    src/comm/server.cpp
    #src/ros/service_stopper.cpp
    #src/ur/commander.cpp
    #src/ur/master_board.cpp
    #src/ur/messages.cpp
    #src/ur/robot_mode.cpp
    #src/ur/rt_state.cpp
    src/primary/primary_package.cpp
    src/primary/robot_message.cpp
    src/primary/robot_message/version_message.cpp
    src/primary/robot_state/kinematics_info.cpp
    src/rtde/control_package_pause.cpp
    src/rtde/control_package_setup_inputs.cpp
    src/rtde/control_package_setup_outputs.cpp
    src/rtde/control_package_start.cpp
    src/rtde/data_package.cpp
    src/rtde/get_urcontrol_version.cpp
    src/rtde/request_protocol_version.cpp
    src/rtde/rtde_package.cpp
    src/rtde/text_message.cpp
    src/rtde/rtde_client.cpp
    src/ur/ur_driver.cpp
)
target_link_libraries(ur_rtde_driver ${catkin_LIBRARIES})
add_dependencies(ur_rtde_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ur_rtde_driver_node
  src/ros/hardware_interface.cpp
  src/ros/hardware_interface_node.cpp
)
target_link_libraries(ur_rtde_driver_node ${catkin_LIBRARIES} ur_rtde_driver)
add_dependencies(ur_rtde_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(plain_driver
  src/main_plain_driver.cpp
)
target_link_libraries(plain_driver ${catkin_LIBRARIES} ur_rtde_driver)
add_dependencies(plain_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(calibration_correction
  src/calibration.cpp
  src/calibration_correction.cpp
)

add_dependencies(calibration_correction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(calibration_correction
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(calibration_test
    test/calibration_test.cpp
    src/calibration.cpp)
  target_link_libraries(calibration_test ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

  #catkin_add_gtest(ur_rtde_driver_test ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_TEST_SOURCES} tests/main.cpp)
  #target_link_libraries(ur_rtde_driver_test ur_rtde_driver ${catkin_LIBRARIES})
#endif()
install(TARGETS calibration_correction RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ur_rtde_driver ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES Apache-2.0_ur_modern_driver.txt DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/LICENSES)
install(FILES manifest.yaml rosdoc.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc)
