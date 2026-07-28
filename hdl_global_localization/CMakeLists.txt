cmake_minimum_required(VERSION 3.0.2)
project(hdl_global_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_service_files(
  FILES
  QueryGlobalLocalization.srv
  SetGlobalLocalizationEngine.srv
  SetGlobalMap.srv
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hdl_global_localization
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(hdl_global_localization
  src/hdl_global_localization/bbs/bbs_localization.cpp
  src/hdl_global_localization/engines/global_localization_bbs.cpp
  src/hdl_global_localization/engines/global_localization_fpfh_ransac.cpp
  src/hdl_global_localization/ransac/ransac_pose_estimation.cpp
  src/hdl_global_localization/ransac/voxelset.cpp
)
add_dependencies(hdl_global_localization
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(hdl_global_localization
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(hdl_global_localization_node
  src/hdl_global_localization_node.cpp
)
add_dependencies(hdl_global_localization_node
  hdl_global_localization
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(hdl_global_localization_node
  hdl_global_localization
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    hdl_global_localization
    hdl_global_localization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
