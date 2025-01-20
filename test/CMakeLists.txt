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