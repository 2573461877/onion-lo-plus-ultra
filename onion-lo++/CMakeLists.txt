cmake_minimum_required(VERSION 2.8.3)
project(onion_lo_plus)

# Set build type and C++ standard
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++17 -O3 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Find Boost libraries
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
if(Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
    LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
endif()

# Find catkin packages
find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    nav_msgs
    sensor_msgs
    roscpp
    rosbag
    std_msgs
    std_srvs
    octomap_msgs
    pcl_conversions
    tf2
    tf2_ros
    livox_ros_driver2
)

catkin_package()

# Find other necessary packages
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3)
find_package(Sophus REQUIRED)
find_package(octomap REQUIRED)
find_package(Ceres REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb tbbmalloc)

# Set C++ and C flags for OpenMP
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# Find OpenMP
FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    # cmake only checks for separate OpenMP library on AppleClang 7+
    if (CMAKE_CXX_COMPILER_ID MATCHES "AppleClang" AND (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "7.0"))
        SET(OpenMP_LIBS ${OpenMP_libomp_LIBRARY})
        LIST(APPEND OpenMVS_EXTRA_LIBS ${OpenMP_LIBS})
    endif()
else()
    message("-- Can't find OpenMP. Continuing without it.")
endif()

# Find OpenMVS (if applicable)
if(OpenMVS_FOUND)
    include_directories(${OpenMVS_INCLUDE_DIRS})
    add_definitions(${OpenMVS_DEFINITIONS})
endif()

# Include directories for catkin and other dependencies
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${livox_ros_driver2_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${TBB_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ./include/
)

# Link directories for Octomap
link_directories(${OCTOMAP_LIBRARY_DIRS})

# Create the executable target
add_executable(onion_lo_plus_node 
    src/ros_main.cpp 
    src/traj_odom.cpp
    src/map_manager.cpp
)

add_dependencies(onion_lo_plus_node ${catkin_EXPORTED_TARGETS})

# Link libraries
target_link_libraries(onion_lo_plus_node 
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SERIALIZATION_LIBRARY} # serialization
    ${Sophus_INCLUDE_DIRS}
    ${TBB_INCLUDE_DIRS}
    ${TBB_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${CERES_LIBRARIES}
    octomath
    pcl_common 
    pcl_io
    fmt::fmt
    TBB::tbb
    TBB::tbbmalloc
)

