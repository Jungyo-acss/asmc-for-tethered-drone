cmake_minimum_required(VERSION 3.0.2)
project(winch_control_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  LIBRARIES winch_control_plugin
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(winch_control_plugin SHARED src/winch_control_plugin.cc)
target_link_libraries(winch_control_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS winch_control_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

