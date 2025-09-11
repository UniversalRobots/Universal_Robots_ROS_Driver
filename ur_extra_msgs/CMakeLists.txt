cmake_minimum_required(VERSION 2.8.3)
project(ur_extra_msgs)

set(MESSAGE_DEPENDENCIES
  std_msgs
  ur_msgs
)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    ${MESSAGE_DEPENDENCIES}
)

add_message_files(
  DIRECTORY msg/
)

generate_messages(
  DEPENDENCIES
    ${MESSAGE_DEPENDENCIES}
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    ${MESSAGE_DEPENDENCIES}
)
