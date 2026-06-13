cmake_minimum_required(VERSION 3.0.2)
project(zlac8015d_canopen_noetic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(zlac8015d_canopen_node
  src/zlac8015d_canopen_node.cpp
)

target_link_libraries(zlac8015d_canopen_node
  ${catkin_LIBRARIES}
)

install(TARGETS zlac8015d_canopen_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
