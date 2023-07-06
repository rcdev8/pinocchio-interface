cmake_minimum_required(VERSION 3.0.2)
project(pinocchio-interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pinocchio-interface
#  CATKIN_DEPENDS other_catkin_pkg
 DEPENDS EIGEN3 pinocchio
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/robot.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)
