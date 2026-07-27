cmake_minimum_required(VERSION 3.10)
project(onion_relocalization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  hdl_global_localization
  nav_msgs
  onion_cloud_preprocessing
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(TBB REQUIRED)
find_library(FLANN_CPP_LIBRARY flann_cpp REQUIRED)
find_library(LZ4_LIBRARY lz4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    cv_bridge geometry_msgs hdl_global_localization nav_msgs
    onion_cloud_preprocessing pcl_conversions pcl_ros rosbag roscpp
    sensor_msgs std_msgs std_srvs tf2_ros
)

set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

add_library(pmc_vendor STATIC
  ${THIRD_PARTY_DIR}/pmc/pmc_heu.cpp
  ${THIRD_PARTY_DIR}/pmc/pmc_maxclique.cpp
  ${THIRD_PARTY_DIR}/pmc/pmcx_maxclique.cpp
  ${THIRD_PARTY_DIR}/pmc/pmcx_maxclique_basic.cpp
  ${THIRD_PARTY_DIR}/pmc/pmc_cores.cpp
  ${THIRD_PARTY_DIR}/pmc/pmc_utils.cpp
  ${THIRD_PARTY_DIR}/pmc/pmc_graph.cpp
  ${THIRD_PARTY_DIR}/pmc/pmc_clique_utils.cpp
)
target_include_directories(pmc_vendor PUBLIC
  ${THIRD_PARTY_DIR}/pmc/include
)
target_link_libraries(pmc_vendor PUBLIC OpenMP::OpenMP_CXX)
target_compile_options(pmc_vendor PRIVATE
  -Wno-unused-function -Wno-sign-compare -Wno-format
)

file(GLOB ROBIN_VENDOR_SOURCES CONFIGURE_DEPENDS
  ${THIRD_PARTY_DIR}/robin/src/*.cpp
)
add_library(robin_vendor STATIC ${ROBIN_VENDOR_SOURCES})
target_include_directories(robin_vendor PUBLIC
  ${THIRD_PARTY_DIR}/robin/include
  ${THIRD_PARTY_DIR}/xenium
)
target_link_libraries(robin_vendor
  PUBLIC Eigen3::Eigen
  PRIVATE pmc_vendor OpenMP::OpenMP_CXX
)

add_library(kiss_matcher_vendor STATIC
  ${THIRD_PARTY_DIR}/kiss_matcher/kiss_matcher/ROBINMatching.cpp
  ${THIRD_PARTY_DIR}/kiss_matcher/kiss_matcher/FasterPFH.cpp
  ${THIRD_PARTY_DIR}/kiss_matcher/kiss_matcher/KISSMatcher.cpp
  ${THIRD_PARTY_DIR}/kiss_matcher/kiss_matcher/GncSolver.cpp
)
target_include_directories(kiss_matcher_vendor PUBLIC
  ${THIRD_PARTY_DIR}/kiss_matcher
)
target_link_libraries(kiss_matcher_vendor PUBLIC
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  TBB::tbb
  robin_vendor
  ${FLANN_CPP_LIBRARY}
  ${LZ4_LIBRARY}
)

add_executable(onion_scancontext_relocalization_node
  src/onion_scancontext_relocalization_node.cpp
  third_party/scancontext/Scancontext.cpp
)
add_dependencies(onion_scancontext_relocalization_node
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(onion_scancontext_relocalization_node PRIVATE
  include
  ${THIRD_PARTY_DIR}/scancontext
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(onion_scancontext_relocalization_node
  kiss_matcher_vendor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(onion_scancontext_database_builder
  src/onion_scancontext_database_builder.cpp
  third_party/scancontext/Scancontext.cpp
)
add_dependencies(onion_scancontext_database_builder
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(onion_scancontext_database_builder PRIVATE
  include
  ${THIRD_PARTY_DIR}/scancontext
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(onion_scancontext_database_builder
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(onion_hdl_relocalization_node
  src/onion_hdl_relocalization_node.cpp
)
add_dependencies(onion_hdl_relocalization_node
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(onion_hdl_relocalization_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(onion_hdl_relocalization_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS
    onion_hdl_relocalization_node
    onion_scancontext_relocalization_node
    onion_scancontext_database_builder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md THIRD_PARTY_NOTICES.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/publish_reference_odometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
