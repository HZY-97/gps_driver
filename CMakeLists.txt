cmake_minimum_required(VERSION 3.8)
project(gps_driver)
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -pthread -fopenmp")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS "-O0 -Wall -Wextra -g -pthread")
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

set(${PROJECT_NAME}_MAJOR_VERSION 1)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_PATCH_VERSION 0)
set(${PROJECT_NAME}_VERSION
  ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})

option(BUILD_DDS_VERSION "use dds instead of ros2" OFF)
option(BUILD_WITH_CCACHE "Build with ccache" ON)
option(BUILD_LINK_WITH_MOLD "linker use mold" ON)
option(USE_TEST "use test" ON)

# 使用DDS全局加宏替换代码
if(BUILD_DDS_VERSION)
  add_definitions(-DUSE_DDS)
  include_directories(${PROJECT_SOURCE_DIR}/dds)

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    message(STATUS "Building for Linux on x86_64.")
    set(DDS_LIB ${PROJECT_SOURCE_DIR}/dds/dds22.04/libdds_util.so)
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message(STATUS "Building for Linux on aarch64.")
    set(DDS_LIB ${PROJECT_SOURCE_DIR}/dds/ddsmrc05/libdds_util.so)
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    message(STATUS "Building for Windows.")
    set(DDS_LIB ${PROJECT_SOURCE_DIR}/dds/windows_dds/dds_util.dll)
  endif()
endif()

if(BUILD_LINK_WITH_MOLD)
  find_program(MOLD_EXECUTABLE mold)
  if(MOLD_EXECUTABLE)
    message(STATUS "Mold linker found at: ${MOLD_EXECUTABLE}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=mold")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fuse-ld=mold")
  else()
    message(WARNING "Mold linker not found, using default linker.")
  endif()
endif()

# Support ccache, if installed
if(NOT MSVC AND NOT XCODE_VERSION)
  if(CCACHE_FOUND)
    message(STATUS "Looking for ccache")
    find_program(CCACHE_FOUND ccache)
    message(STATUS "Found ccache: ${CCACHE_FOUND}")
    if(BUILD_WITH_CCACHE)
      message(STATUS "USE ccache")
      set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
      set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    else()
      set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "")
      set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "")
    endif()
  endif(CCACHE_FOUND)
endif()

if(NOT BUILD_DDS_VERSION)
  #---------for msg---------
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  include_directories(
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
  )

  # for rclcpp
  find_package(rclcpp REQUIRED)
  include_directories(${rclcpp_INCLUDE_DIRS})

endif()

find_package(glog REQUIRED)

add_subdirectory(src)

if(USE_TEST)
  add_subdirectory(test)
endif()

install(
  FILES
      ${PROJECT_SOURCE_DIR}/shell/setenv.bash
      ${PROJECT_SOURCE_DIR}/shell/gps_driver.bash
  DESTINATION .
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
              GROUP_EXECUTE GROUP_READ
              WORLD_EXECUTE WORLD_READ
)

