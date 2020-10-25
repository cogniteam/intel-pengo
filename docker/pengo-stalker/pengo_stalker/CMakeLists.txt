cmake_minimum_required(VERSION 3.0.2)
project(pengo_stalker)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  kobuki_msgs
  nav_msgs
  roscpp
  tf
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pengo_stalker_node 
  src/pengo_stalker_node.cpp
  src/PengoStalkerRos.cpp
)

add_dependencies(pengo_stalker_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pengo_stalker_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS pengo_stalker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
