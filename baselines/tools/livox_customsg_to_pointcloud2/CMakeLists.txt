cmake_minimum_required(VERSION 3.5)
project(livox_customsg_to_pointcloud2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  livox_ros_driver
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs livox_ros_driver
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(converter_node src/converter_node.cpp)
add_dependencies(converter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(converter_node ${catkin_LIBRARIES})

install(TARGETS converter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
