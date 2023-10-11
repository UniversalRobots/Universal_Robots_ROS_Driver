cmake_minimum_required(VERSION 3.0.2)
project(ur_robot_driver)

add_definitions( -DROS_BUILD )

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Default to off as this only works in environments where a new docker container can be started with the appropriate networking, which
# might not be possible e.g. inside the buildfarm
option(
  UR_ROBOT_DRIVER_BUILD_INTEGRATION_TESTS
  "Build integration tests using the start_ursim script"
  OFF
)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    control_msgs
    controller_manager
    controller_manager_msgs
    geometry_msgs
    hardware_interface
    industrial_robot_status_interface
    pluginlib
    roscpp
    scaled_joint_trajectory_controller
    sensor_msgs
    speed_scaling_interface
    speed_scaling_state_controller
    std_srvs
    tf
    tf2_geometry_msgs
    tf2_msgs
    trajectory_msgs
    ur_dashboard_msgs
    ur_msgs
    pass_through_controllers
    kdl_parser
)
find_package(Boost REQUIRED)
find_package(ur_client_library REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ur_robot_driver_plugin
    urcl_log_handler
  CATKIN_DEPENDS
    actionlib
    control_msgs
    controller_manager
    controller_manager_msgs
    geometry_msgs
    hardware_interface
    kdl_parser
    pass_through_controllers
    pluginlib
    roscpp
    scaled_joint_trajectory_controller
    sensor_msgs
    speed_scaling_interface
    speed_scaling_state_controller
    std_srvs
    tf
    tf2_geometry_msgs
    tf2_msgs
    trajectory_msgs
    ur_dashboard_msgs
    ur_msgs
  DEPENDS
    Boost
    ur_client_library
)

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

add_library(urcl_log_handler
  src/urcl_log_handler.cpp
)
target_link_libraries(urcl_log_handler ${catkin_LIBRARIES} ur_client_library::urcl)

add_executable(ur_robot_driver_node
  src/dashboard_client_ros.cpp
  src/hardware_interface.cpp
  src/hardware_interface_node.cpp
)
target_link_libraries(ur_robot_driver_node ${catkin_LIBRARIES} ur_client_library::urcl urcl_log_handler)
add_dependencies(ur_robot_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(dashboard_client
  src/dashboard_client_ros.cpp
  src/dashboard_client_node.cpp
)
target_link_libraries(dashboard_client ${catkin_LIBRARIES} ur_client_library::urcl urcl_log_handler)
add_dependencies(dashboard_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(robot_state_helper
  src/robot_state_helper.cpp
  src/robot_state_helper_node.cpp
)
target_link_libraries(robot_state_helper ${catkin_LIBRARIES} ur_client_library::urcl)
add_dependencies(robot_state_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(controller_stopper_node
  src/controller_stopper.cpp
  src/controller_stopper_node.cpp
)
add_dependencies(controller_stopper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_stopper_node
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  if(${UR_ROBOT_DRIVER_BUILD_INTEGRATION_TESTS})
    add_rostest(test/driver.test)
  endif()
endif()


install(TARGETS ur_robot_driver_plugin urcl_log_handler ur_robot_driver_node robot_state_helper dashboard_client controller_stopper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/tool_communication scripts/test_move
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
