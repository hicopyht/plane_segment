cmake_minimum_required(VERSION 2.8.3)
project(plane_segment)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

SET( CMAKE_BUILD_TYPE RelWithDebInfo )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_filters
  pcl_ros
  rosbag
  roscpp
  sensor_msgs
  cv_bridge
  tf
  plane_from_line
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)


#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/PlaneSegment.cfg
  cfg/LineBasedSegment.cfg
  cfg/RansacSegment.cfg
  cfg/OrganizedSegment.cfg
  cfg/RegionGrowSegment.cfg
  cfg/Viewer.cfg
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plane_segment
#  CATKIN_DEPENDS dynamic_reconfigure message_filters pcl_ros roscpp sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/plane_segment.cpp
  src/organized_plane_segment.cpp
  src/plane_viewer.cpp
  src/utils.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBS}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

## Declare a C++ executable
add_executable(plane_segment_node src/plane_segment_node.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(plane_segment_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(plane_segment_node ${PROJECT_NAME}_gencfg)
target_link_libraries(plane_segment_node
    ${PROJECT_NAME}
)

## Declare a C++ executable
add_executable(plane_segment_bagfile src/plane_segment_bagfile.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(plane_segment_bagfile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(plane_segment_bagfile ${PROJECT_NAME}_gencfg)
target_link_libraries(plane_segment_bagfile
    ${PROJECT_NAME}
)

## Declare a C++ executable
add_executable(plane_segment_pcd src/plane_segment_pcb.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(plane_segment_pcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(plane_segment_pcd ${PROJECT_NAME}_gencfg)
target_link_libraries(plane_segment_pcd
    ${PROJECT_NAME}
)

## Declare a C++ executable
add_executable(plane_segment_bagfile_depth src/plane_segment_bagfile_depth.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(plane_segment_bagfile_depth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(plane_segment_bagfile_depth ${PROJECT_NAME}_gencfg)
target_link_libraries(plane_segment_bagfile_depth
    ${PROJECT_NAME}
)
