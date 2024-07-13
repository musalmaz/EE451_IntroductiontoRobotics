cmake_minimum_required(VERSION 3.0.2)
project(project_ppprobot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  roscpp
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES project_ppprobot_library
  CATKIN_DEPENDS controller_manager joint_state_controller robot_state_publisher roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(project_ppprobot_library src/spawn_cube.cpp src/control_robot.cpp)
target_link_libraries(project_ppprobot_library ${catkin_LIBRARIES})

add_executable(spawn src/spawn_cube_main.cpp)
target_link_libraries(spawn project_ppprobot_library)

add_executable(main src/main.cpp)
target_link_libraries(main project_ppprobot_library)