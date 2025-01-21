add_executable(tcpClient tcpClient.cpp)
install(
    TARGETS tcpClient
    DESTINATION test
)

set_target_properties(
    tcpClient
    PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib/"
)

add_executable(udp_server udp_server.cpp)
install(
    TARGETS udp_server
    DESTINATION test
)

set_target_properties(
    udp_server
    PROPERTIES
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib/"
)