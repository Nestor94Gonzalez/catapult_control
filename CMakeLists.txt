cmake_minimum_required(VERSION 2.8.3)
project(catapult_control)

SET(CMAKE_CXX_FLAGS "-std=c++11")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  std_msgs
  message_generation
)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

# Depend on system install of Gazebo
#find_package(gazebo REQUIRED)

#link_directories(${GAZEBO_LIBRARY_DIRS})
#include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
    Catapult.msg
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    message_runtime
)


###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/catapult_plugin.cc
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)