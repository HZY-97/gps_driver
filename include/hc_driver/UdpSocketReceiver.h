#ifndef UDP_SOCKET_RECEIVER_H
#define UDP_SOCKET_RECEIVER_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "siasunLog.h"


class UdpSocketReceiver {
public:
    UdpSocketReceiver(int port);
    ~UdpSocketReceiver();

    bool Initialize();
    bool ReceiveData(char* buffer, int bufferSize);

private:
    int sockfd;
    int bindPort;
    struct sockaddr_in servaddr;
};

#endif  // UDP_SOCKET_RECEIVER_H

// int main() {
//     const int BUFFER_SIZE = 1024;
//     const int UDP_PORT = 9902;

//     UdpSocketReceiver udpReceiver(UDP_PORT);
//     if (!udpReceiver.Initialize()) {
//         std::cerr << "Failed to initialize UDP socket" << std::endl;
//         return 1;
//     }

//     char buffer[BUFFER_SIZE];
//     while (true) {
//         if (udpReceiver.ReceiveData(buffer, BUFFER_SIZE)) {
//             // 处理接收到的数据
//             std::cout << "Received data: " << buffer << std::endl;
//         }
//     }

//     return 0;
// }