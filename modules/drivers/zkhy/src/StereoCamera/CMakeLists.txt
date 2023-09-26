cmake_minimum_required(VERSION 2.8)

project(StereoCamera)
# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs rosnode)
catkin_package()
include_directories("../inc" ${catkin_INCLUDE_DIRS})

message("Current folder is: ${CMAKE_CURRENT_LIST_DIR}")

if (WIN32)
  # set stuff for windows
  if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Only x64 is supported!\n please use '-G 'Visual Studio 15 2017 Win64'' as argument")
  endif()
else()
  # set stuff for other systems
  add_definitions(-std=c++11)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../DBin")
else()
  set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../Bin")
endif()

link_directories("${BIN_DIR}")
link_libraries("${BIN_DIR}/libStereoCamera.so" ImageUtils boost_system)
############################################################
# Create an executable
############################################################

# Add an executable with the above sources
add_executable(StereoCamera
  main.cpp camera_handler.cpp
)
target_link_libraries(StereoCamera ${catkin_LIBRARIES})

set_target_properties(StereoCamera
  PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY "${BIN_DIR}"
  LIBRARY_OUTPUT_DIRECTORY "${BIN_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
)
