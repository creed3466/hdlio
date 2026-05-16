cmake_minimum_required(VERSION 3.5)
project(livox_v2_to_v1_republish)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  topic_tools
  livox_ros_driver
)

catkin_package(
  CATKIN_DEPENDS roscpp topic_tools livox_ros_driver
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(republish_node src/republish_node.cpp)
add_dependencies(republish_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(republish_node
  ${catkin_LIBRARIES}
)

install(TARGETS republish_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
