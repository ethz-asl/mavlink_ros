# Declare the version of the CMake API for forward compatibility
cmake_minimum_required(VERSION 2.8.3)

# Declare the name of the CMake Project
project(mavlink_ros)

find_package(catkin_simple REQUIRED)
catkin_simple()

SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

include(cmake/FindMAVLINK.cmake)
include(cmake/FindGLIB2.cmake)
include(cmake/FindGTHREAD2.cmake)

INCLUDE_DIRECTORIES(
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ../include/
)

find_package(GLIB2  REQUIRED)
find_package(GTHREAD2  REQUIRED)

# Declare an executable and specify libraries to link against
cs_add_executable(mavlink_ros_serial src/mavlink_ros_serial.cpp)
target_link_libraries(mavlink_ros_serial 
                      ${GLIB2_LIBRARY}
                      ${GTHREAD2_LIBRARY})

cs_install()
cs_export()
