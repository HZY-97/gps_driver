include_directories(
  ${PROJECT_SOURCE_DIR}/include/gps_driver
  ${PROJECT_SOURCE_DIR}/thridparty
  ${CMAKE_CURRENT_SOURCE_DIR}
)

if(NOT BUILD_DDS_VERSION)
    set(GPS_DRIVER_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/ros_gps_driver_node.cpp
    )
    add_executable(${PROJECT_NAME}_node ${GPS_DRIVER_SRC})

    target_link_libraries(${PROJECT_NAME}_node 
        ${rclcpp_LIBRARIES}
        ${cpp_typesupport_target}
        ${nav_msgs_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        glog
    )

    install(
        TARGETS ${PROJECT_NAME}_node 
        DESTINATION bin
    )

    set_target_properties(
        ${PROJECT_NAME}_node 
        PROPERTIES
        INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib/"
    )
else()
endif()



