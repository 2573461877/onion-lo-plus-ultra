cmake_minimum_required(VERSION 3.10)
project(onion_gps_evaluation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES onion_gps_evaluation_core
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
  DEPENDS EIGEN3
)

add_library(onion_gps_evaluation_core
  src/trajectory_alignment.cpp
  src/evaluation_runner.cpp
)
target_compile_features(onion_gps_evaluation_core PUBLIC cxx_std_17)
target_compile_options(onion_gps_evaluation_core PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)
target_include_directories(onion_gps_evaluation_core
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(onion_gps_evaluation_core
  PUBLIC
    Eigen3::Eigen
    ${catkin_LIBRARIES}
)
add_dependencies(
  onion_gps_evaluation_core
  ${catkin_EXPORTED_TARGETS}
)

function(add_gps_evaluation_node target source)
  add_executable(${target} ${source})
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(${target}
    PRIVATE
      onion_gps_evaluation_core
      ${catkin_LIBRARIES}
  )
  add_dependencies(${target} ${catkin_EXPORTED_TARGETS})
endfunction()

add_gps_evaluation_node(
  trajectory_registration_node
  src/trajectory_registration_node.cpp
)
add_gps_evaluation_node(
  localization_accuracy_evaluator_node
  src/localization_accuracy_evaluator_node.cpp
)
add_gps_evaluation_node(
  segmented_registration_evaluator_node
  src/segmented_registration_evaluator_node.cpp
)
add_gps_evaluation_node(
  synthetic_trajectory_publisher_node
  src/synthetic_trajectory_publisher_node.cpp
)

install(
  TARGETS
    onion_gps_evaluation_core
    trajectory_registration_node
    localization_accuracy_evaluator_node
    segmented_registration_evaluator_node
    synthetic_trajectory_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  PROGRAMS scripts/play_bag_and_finalize.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    trajectory_alignment_test
    test/trajectory_alignment_test.cpp
  )
  if(TARGET trajectory_alignment_test)
    target_compile_features(trajectory_alignment_test PRIVATE cxx_std_17)
    target_link_libraries(
      trajectory_alignment_test
      onion_gps_evaluation_core
    )
  endif()
endif()
