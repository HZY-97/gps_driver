/**
 * @file tcpClient.cpp
 * @author huizeyu
 * @brief
 * @version 0.1
 * @date 2023-08-01
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

const int SERVER_PORT = 9902;
const char *SERVER_IP = "127.0.0.1";  // 修改为服务器IP地址
const int MESSAGE_SIZE = 100;  // 每次发送的数据大小，这里设为100字节

int main() {
  int clientSocket =
      socket(AF_INET, SOCK_STREAM, 0);  // 创建套接字，返回套接字描述符
  if (clientSocket == -1) {
    std::cerr << "Error creating socket." << std::endl;
    return 1;
  }

  sockaddr_in serverAddr{};         // 创建服务器地址结构体
  serverAddr.sin_family = AF_INET;  // 使用IPv4地址族
  serverAddr.sin_port =
      htons(SERVER_PORT);  // 设置端口号，使用网络字节序（大端序）
  inet_pton(AF_INET, SERVER_IP,
            &(serverAddr.sin_addr));  // 将IP地址字符串转换为二进制格式

  while (connect(clientSocket, (struct sockaddr *)&serverAddr,
                 sizeof(serverAddr)) == -1) {
    std::cerr << "Error connecting to the server." << std::endl;
    close(clientSocket);
    usleep(50 * 1000);  // 等待500ms再重连
  }

  while (true) {
    // 模拟发送数据
    static int a = 1;
    std::string message = "Data sent : " + std::to_string(a++);
    char buffer[MESSAGE_SIZE] = {0};
    strncpy(buffer, message.c_str(), MESSAGE_SIZE);  // 将消息复制到发送缓冲区
    send(clientSocket, buffer, MESSAGE_SIZE, 0);    // 发送数据
    std::cout << "Sent: " << message << std::endl;  // 打印已发送的消息

    usleep(100 * 1000);  // 等待100ms再发送
  }

  close(clientSocket);  // 关闭套接字
  return 0;
}
