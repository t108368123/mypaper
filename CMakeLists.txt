cmake_minimum_required(VERSION 3.0.2)
project(mypaper)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++14)

set(DEFAULT_CXX_STANDARD ${CMAKE_CXX_STANDARD})
if (NOT DEFAULT_CXX_STANDARD)
  set(DEFAULT_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD ${DEFAULT_CXX_STANDARD} CACHE STRING
  "C++ standard (minimum 14)" FORCE)
# Restrict CMAKE_CXX_STANDARD to the valid versions permitted and ensure that
# if one was forced via -D that it is in the valid set.
set(ALLOWED_CXX_STANDARDS 14 17 20)
set_property(CACHE CMAKE_CXX_STANDARD PROPERTY STRINGS ${ALLOWED_CXX_STANDARDS})
list(FIND ALLOWED_CXX_STANDARDS ${CMAKE_CXX_STANDARD} POSITION)
if (POSITION LESS 0)
  message(FATAL_ERROR "Invalid CMAKE_CXX_STANDARD: ${CMAKE_CXX_STANDARD}. "
    "Must be one of: ${ALLOWED_CXX_STANDARDS}")
endif()
# Specify the standard as a hard requirement, otherwise CMAKE_CXX_STANDARD is
# interpreted as a suggestion that can decay *back* to lower versions.
set(CMAKE_CXX_STANDARD_REQUIRED ON CACHE BOOL "")
mark_as_advanced(CMAKE_CXX_STANDARD_REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  pcl_conversions
  pcl_ros
  geometry_msgs
  image_transport
  cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.5 REQUIRED)

catkin_package(
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(hough-3d-lines
  include/hough-3d-lines/hough.cpp
  include/hough-3d-lines/hough3dlines.cpp
  include/hough-3d-lines/pointcloud.cpp
  include/hough-3d-lines/sphere.cpp
  include/hough-3d-lines/vector3d.cpp
        )

add_executable(mypaper src/feature_extractor.cpp)
target_link_libraries(mypaper hough-3d-lines 
                      ${catkin_LIBRARIES} 
                      ${OpenCV_LIBS})


add_executable(mypaper_two src/feature_extractor_two.cpp)
target_link_libraries(mypaper_two hough-3d-lines 
                      ${catkin_LIBRARIES} 
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES})

add_executable(mypaper_one src/feature_extractor.cpp)
target_link_libraries(mypaper_one hough-3d-lines 
                      ${catkin_LIBRARIES} 
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES})
