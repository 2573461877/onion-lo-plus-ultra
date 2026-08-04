cmake_minimum_required(VERSION 3.0.2)
project(fast_lio_localization_humanoid)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-O3 -pthread -fexceptions)

include(ProcessorCount)
ProcessorCount(FAST_LIO_PROCESSOR_COUNT)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  if(FAST_LIO_PROCESSOR_COUNT GREATER 4)
    add_compile_definitions(MP_EN MP_PROC_NUM=3)
  elseif(FAST_LIO_PROCESSOR_COUNT GREATER 3)
    add_compile_definitions(MP_EN MP_PROC_NUM=2)
  else()
    add_compile_definitions(MP_PROC_NUM=1)
  endif()
else()
  # Keep the upstream conservative setting on ARM until the target is
  # benchmarked; OpenMP is still linked for omp_get_wtime().
  add_compile_definitions(MP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  livox_ros_driver2
  message_generation
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  urdf
  visualization_msgs
)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include include/fast_lio
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    livox_ros_driver2
    message_runtime
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_ros
    urdf
    visualization_msgs
  DEPENDS Boost EIGEN3 PCL
)

include_directories(
  include
  include/fast_lio
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(fast_lio_humanoid_mapping
  src/fast_lio/laserMapping.cpp
  src/fast_lio/preprocess.cpp
  include/fast_lio/ikd-Tree/ikd_Tree.cpp
)
add_dependencies(fast_lio_humanoid_mapping
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fast_lio_humanoid_mapping
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(fast_lio_humanoid_mapping OpenMP::OpenMP_CXX)
endif()

option(BUILD_OPEN3D_LOCALIZATION
  "Build the Open3D fixed-map localization node" ON)

if(BUILD_OPEN3D_LOCALIZATION)
  find_package(Open3D QUIET CONFIG)
  if(Open3D_FOUND)
    if(TARGET Open3D::Open3D)
      set(FAST_LIO_OPEN3D_TARGET Open3D::Open3D)
    else()
      set(FAST_LIO_OPEN3D_TARGET ${Open3D_LIBRARIES})
    endif()

    add_library(fast_lio_humanoid_open3d_registration
      src/open3d_loc/open3d_registration/open3d_registration.cpp
    )
    target_include_directories(fast_lio_humanoid_open3d_registration PRIVATE
      ${Open3D_INCLUDE_DIRS}
    )
    target_link_libraries(fast_lio_humanoid_open3d_registration
      ${FAST_LIO_OPEN3D_TARGET}
    )

    add_library(fast_lio_humanoid_open3d_conversions
      src/open3d_loc/open3d_conversions/open3d_conversions.cpp
    )
    target_include_directories(fast_lio_humanoid_open3d_conversions PRIVATE
      ${Open3D_INCLUDE_DIRS}
    )
    target_link_libraries(fast_lio_humanoid_open3d_conversions
      ${FAST_LIO_OPEN3D_TARGET}
      ${catkin_LIBRARIES}
    )

    add_executable(fast_lio_humanoid_localization
      src/open3d_loc/global_localization.cpp
    )
    add_dependencies(fast_lio_humanoid_localization
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_include_directories(fast_lio_humanoid_localization PRIVATE
      ${Open3D_INCLUDE_DIRS}
    )
    target_link_libraries(fast_lio_humanoid_localization
      fast_lio_humanoid_open3d_conversions
      fast_lio_humanoid_open3d_registration
      ${Boost_LIBRARIES}
      ${FAST_LIO_OPEN3D_TARGET}
      ${catkin_LIBRARIES}
    )

    install(TARGETS
      fast_lio_humanoid_localization
      fast_lio_humanoid_open3d_conversions
      fast_lio_humanoid_open3d_registration
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    message(STATUS
      "FAST_LIO_LOCALIZATION_HUMANOID: Open3D localization enabled")
  else()
    message(WARNING
      "Open3D was not found. Mapping will be built, but fixed-map "
      "localization is disabled. Set Open3D_DIR and rebuild to enable it.")
  endif()
endif()

install(TARGETS fast_lio_humanoid_mapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md UPSTREAM.md DRIVER_COMPATIBILITY.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY LICENSES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
