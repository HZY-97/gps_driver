include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src/libParamLoader
    ${PROJECT_SOURCE_DIR}/src/common
)

add_executable(json_sub json_sub.cpp)
target_link_libraries(json_sub
    ${DDS_LIB}
    dl
    # ${rclcpp_LIBRARIES}
)

install(
    TARGETS json_sub
    DESTINATION test
)

set_target_properties(
    json_sub
    PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib/robo_slam_3d"
)

add_executable(json_pub json_pub.cpp)
target_link_libraries(json_pub
    ${DDS_LIB}
    dl
)

install(
    TARGETS json_pub
    DESTINATION test
)

set_target_properties(
    json_pub
    PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib/robo_slam_3d"
)

add_executable(pcd_pub pcd_pub.cpp)
target_link_libraries(pcd_pub
    ${DDS_LIB}
    dl
    ${PCL_LIBRARIES}
)

install(
    TARGETS pcd_pub
    DESTINATION test
)

set_target_properties(
    pcd_pub
    PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib/robo_slam_3d"
)

add_executable(pcd_sub pcd_sub.cpp)
target_link_libraries(pcd_sub
    ${DDS_LIB}
    dl
    ${PCL_LIBRARIES}
)

install(
    TARGETS pcd_sub
    DESTINATION test
)

set_target_properties(
    pcd_sub
    PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib/robo_slam_3d"
)