cmake_minimum_required(VERSION 2.8.3)
project(ur_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ur_robot_driver
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# create alias for the '_INCLUDE_DIR' variable, to help Catkin export it
set(YAML_CPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    ur_robot_driver
  DEPENDS
    YAML_CPP
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(calibration_correction
  src/calibration.cpp
  src/calibration_consumer.cpp
  src/calibration_correction.cpp
)

add_dependencies(calibration_correction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(calibration_correction
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(calibration_test
    test/calibration_test.cpp
    src/calibration.cpp)
  target_link_libraries(calibration_test ${catkin_LIBRARIES} yaml-cpp)
endif()

install(TARGETS calibration_correction RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
