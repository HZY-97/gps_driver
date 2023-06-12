cmake_minimum_required(VERSION 3.0.2)
project(hc_driver)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DOPEN_LOG)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hc_driver
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/hc_driver
)


aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src HC_DRIVER_SRC)
add_executable(${PROJECT_NAME}_node ${HC_DRIVER_SRC})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})