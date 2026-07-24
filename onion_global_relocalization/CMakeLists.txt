cmake_minimum_required(VERSION 3.0.2)
project(onion_global_relocalization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hdl_global_localization
  pcl_conversions
  roscpp
  sensor_msgs
  std_srvs
)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    hdl_global_localization
    pcl_conversions
    roscpp
    sensor_msgs
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(onion_global_relocalization_node
  src/onion_global_relocalization_node.cpp
)
add_dependencies(onion_global_relocalization_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(onion_global_relocalization_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS onion_global_relocalization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
