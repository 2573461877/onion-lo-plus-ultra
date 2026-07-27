cmake_minimum_required(VERSION 3.10)
project(onion_cloud_preprocessing LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_ros
  DEPENDS EIGEN3
)

add_executable(vehicle_crop_filter_node
  src/vehicle_crop_filter_node.cpp
)
target_include_directories(vehicle_crop_filter_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(vehicle_crop_filter_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
add_dependencies(vehicle_crop_filter_node ${catkin_EXPORTED_TARGETS})

install(TARGETS vehicle_crop_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
