cmake_minimum_required(VERSION 3.0.2)
project(gps_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hc_driver
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/gps_driver
  ${CMAKE_CURRENT_SOURCE_DIR}/thridparty/easyloggingpp_usage/src
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thridparty/easyloggingpp_usage)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src GPS_DRIVER_SRC)
add_executable(${PROJECT_NAME}_node ${GPS_DRIVER_SRC})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} logTool)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)