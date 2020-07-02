cmake_minimum_required(VERSION 2.8.3)
project(decode_gps)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)
if(NOT ${CMAKE_VERSION} VERSION_LESS 3.0)
    cmake_policy(SET CMP0026 OLD)
endif()
add_compile_options(-std=c++11) 
add_compile_options(-fPIC)

#Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# Find ROS related-packages
find_package(catkin REQUIRED #COMPONENTS
  std_msgs
  sensor_msgs
  nav_msgs
  roscpp
  pcl_ros
  pcl_conversions
  tf
  tf_conversions
  eigen_conversions
  )

#Find Eigen
find_package(Eigen3 REQUIRED)
#Find PCL
find_package(PCL 1.7 REQUIRED common io visualization)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "Using PCL version ${PCL_VERSION}")

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_message_files(
        DIRECTORY msg
        FILES
        gpfpd.msg)

        generate_messages(DEPENDENCIES std_msgs) 
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  INCLUDE_DIRS include
  )
############
# Libraries#
############

add_library(get_gnss SHARED src/gps_msg_process.cpp)
target_link_libraries(get_gnss ${catkin_LIBRARIES})
target_include_directories(get_gnss PUBLIC ${PROJECT_SOURCE_DIR}/include)


#############
#Executables#
#############
add_executable(decode_gps src/decode_gps.cpp)
target_link_libraries(decode_gps ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES} get_gnss)