#include <unistd.h>
#include <cstring>
#include <iostream>

#include "UdpSocketReceiver.h"

UdpSocketReceiver::UdpSocketReceiver(int port) : sockfd(-1), bindPort(port) {}

UdpSocketReceiver::~UdpSocketReceiver() {
    if (sockfd != -1) {
        close(sockfd);
    }
}

bool UdpSocketReceiver::Initialize() {
    // 创建UDP套接字
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }

    // 设置服务器地址和端口
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(bindPort);

    // 绑定套接字到指定端口
    if (bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(sockfd);
        return false;
    }

    return true;
}

bool UdpSocketReceiver::ReceiveData(char* buffer, int bufferSize) {
    memset(buffer, 0, bufferSize);
    ssize_t numBytes = recvfrom(sockfd, buffer, bufferSize, 0, nullptr, nullptr);
    if (numBytes == -1) {
        std::cerr << "Failed to receive data" << std::endl;
        return false;
    }

    return true;
}
