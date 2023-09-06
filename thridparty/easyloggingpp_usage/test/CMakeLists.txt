include_directories(
${CMAKE_CURRENT_SOURCE_DIR}/../src
)


add_executable(testLogTool testLogTool.cpp)
target_link_libraries(testLogTool logTool)