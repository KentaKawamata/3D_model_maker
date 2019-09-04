cmake_minimum_required(VERSION 2.8.3)
project(model_maker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

function(Flag flag init_flag flag_message)
    if(NOT DEFINED ${flag})
        set(${flag} ${init_flag} CACHE BOOL "${flag_message} : true or false" FORCE)
    endif()
endfunction(Flag)

Flag(USE_Boost true "Find and link Boost library")
Flag(USE_CV true "Find and link OpenCV library")
Flag(USE_RS2 true "Find and link librealsense2 library")
Flag(USE_PCL true "Find and link Point Cloud Library")
Flag(USE_EIGEN true "Find and link Eigen Library")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
if(USE_Boost)
    find_package(Boost REQUIRED)

    if(Boost_FOUND)
        message("---- Found Boost ${Boost_INCLUDE_DIRS}")

    else(Boost_FOUND)
        message(FATAL_ERROR " ---- Not Found Boost")

    endif(Boost_FOUND)
endif(USE_Boost)

if(USE_CV)
    find_package(OpenCV REQUIRED)

    if(OpenCV_FOUND)
        message("---- Found OpenCV ${OpenCV_INCLUDE_DIRS}")

    else(OpenCV_FOUND)
        message(FATAL_ERROR "---- Not Found OpenCV")

    endif(OpenCV_FOUND)
endif(USE_CV)

if(USE_RS2)
    find_package(realsense2 REQUIRED)

    if(realsense2_FOUND)
        message("---- Found realsense2 ${realsense2_INCLUDE_DIRS}")

    else(realsense2_FOUND)
        message(FATAL_ERROR "---- Not Found realsense2")

    endif(realsense2_FOUND)
endif(USE_RS2)

if(USE_PCL)
    find_package(PCL REQUIRED)

    if(PCL_FOUND)
        message("---- Found PCL ${PCL_INCLUDE_DIRS}")

    else(PCL_FOUND)
        message(FATAL_ERROR "---- Not Found PCL")

    endif(PCL_FOUND)
endif(USE_PCL)

if(USE_EIGEN)
    find_package(Eigen3 REQUIRED)

    if(Eigen3_FOUND)
        message("---- Found Eigen ${Eigen3_INCLUDE_DIRS}")

    else(Eigen3_FOUND)
        message(FATAL_ERROR "---- Not Found Eigen")

    endif(Eigen3_FOUND)
endif(USE_EIGEN)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp std_msgs tf2_ros
  DEPENDS
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/make_ply.cpp
  include/make_ply.hpp
  src/editCloud.cpp
  include/editCloud.hpp
  src/getRotationVector.cpp
  include/getRotationVector.hpp
)

add_dependencies(${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${realsense2_LIBRARY}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)

