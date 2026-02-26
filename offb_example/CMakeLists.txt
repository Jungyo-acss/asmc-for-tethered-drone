cmake_minimum_required(VERSION 3.0.2)
project(offb_example)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
  std_msgs
  rospy
  gazebo_ros
  std_srvs
)

# Eigen 패키지 찾기
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Ceres package
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# find gazebo
find_package(gazebo REQUIRED)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# gazebo setup
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/PIDPositionControl.cfg
  cfg/ASMCAttitudeControl.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pid_position_control_lib asmc_attitude_control_lib
  CATKIN_DEPENDS 
    geometry_msgs 
    mavros_msgs 
    roscpp 
    tf2 
    tf2_ros 
    tf2_geometry_msgs 
    rospy 
    std_msgs 
    dynamic_reconfigure
)

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIR}
)

## Declare C++ libraries
add_library(pid_position_control_lib
  src/pid_position_control.cpp
)

add_library(asmc_attitude_control_lib
  src/asmc_attitude_control.cpp
)

# add disturb library
add_library(disturbance_plugin SHARED src/disturbance_plugin.cpp)



## Add dependencies to the libraries
add_dependencies(pid_position_control_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_dependencies(asmc_attitude_control_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

## Link libraries
target_link_libraries(pid_position_control_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(asmc_attitude_control_lib
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  glog
)

## add disturb libraries

target_link_libraries(disturbance_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

## Declare executable
add_executable(offb_node 
  src/offb_node.cpp
)

add_executable(pid_position_control_node
  src/pid_position_control_node.cpp
)

add_executable(asmc_attitude_control_node
  src/asmc_attitude_control_node.cpp
)

## Add dependencies to executable
add_dependencies(offb_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_dependencies(pid_position_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_dependencies(asmc_attitude_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

## Link executable with libraries
target_link_libraries(offb_node
  pid_position_control_lib
  asmc_attitude_control_lib
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  glog

)

target_link_libraries(pid_position_control_node
  pid_position_control_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(asmc_attitude_control_node
  asmc_attitude_control_lib
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  glog
)

#############
## Install ##
#############
# Install libraries
install(TARGETS pid_position_control_lib asmc_attitude_control_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install executables
install(TARGETS offb_node pid_position_control_node asmc_attitude_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
