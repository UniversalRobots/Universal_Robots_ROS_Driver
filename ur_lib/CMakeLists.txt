cmake_minimum_required(VERSION 2.8.12)
project(ur_lib)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/" ${CMAKE_MODULE_PATH})
#add_definitions( -DROS_BUILD )

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(Boost REQUIRED)

##
## Check C++11 support / enable global pedantic and Wall
##
include(DefineCXX11CompilerFlag)
DEFINE_CXX_11_COMPILER_FLAG(CXX11_FLAG)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

add_library(ur_lib SHARED
    src/comm/tcp_socket.cpp
    src/comm/server.cpp
    src/primary/primary_package.cpp
    src/primary/robot_message.cpp
    src/primary/robot_state.cpp
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
    src/ur/calibration_checker.cpp
    src/ur/dashboard_client.cpp
    src/ur/tool_communication.cpp
    src/rtde/rtde_writer.cpp
)
add_library(ur_lib::ur_lib ALIAS ur_lib)
target_compile_options(ur_lib PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_compile_options(ur_lib PUBLIC ${CXX11_FLAG})
target_include_directories( ur_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(ur_lib INTERFACE ${Boost_Libraries})

install(TARGETS ur_lib EXPORT ur_lib_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION include)

install(EXPORT ur_lib_targets
  DESTINATION lib/cmake/ur_lib
  FILE ur_libTargets.cmake
  NAMESPACE ur_lib::)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/ur_libConfigVersion.cmake VERSION 0.0.3
  COMPATIBILITY SameMajorVersion)
install(FILES ur_libConfig.cmake
  DESTINATION lib/cmake/ur_lib)

