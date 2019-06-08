cmake_minimum_required(VERSION 2.8.3)
project(smartcar_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
#  serial
)
#寻找多线程库
find_package(Threads )
################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES smartcar_bringup
#  message_generation 
  CATKIN_DEPENDS nav_msgs roscpp rospy  std_msgs tf #serial
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
 include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}
  ${Threads_INCLUDE_DIRS} 
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/base_controller.cpp
#   src/base_controller2.cpp
)

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${Threads_INCLUDE_DIRS} 
)

add_executable(smartcar_node src/nodes/smartcar_node.cpp)
target_link_libraries(smartcar_node ${PROJECT_NAME})

#add_executable(smartcar_node2 src/nodes/smartcar_node2.cpp)
#target_link_libraries(smartcar_node2 ${PROJECT_NAME})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

