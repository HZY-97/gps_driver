add_definitions(-DELPP_STL_LOGGING  )
add_definitions(-DELPP_NO_DEFAULT_LOG_FILE)
add_definitions(-DELPP_FORCE_USE_STD_THREAD)
# add_definitions(-DELPP_THREAD_SAFE)

add_library(logTool STATIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/easylogging++.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/logTool.cpp
)

# add_subdirectory(test)