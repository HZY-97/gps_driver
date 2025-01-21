#include <arpa/inet.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <iostream>

#define PORT 9996
#define BUFFER_SIZE 1024

int main() {
  // 创建 UDP 套接字
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    std::cerr << "无法创建套接字" << std::endl;
    return -1;
  }

  // 设置服务器地址结构
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;  // 监听本机的所有 IP 地址
  server_addr.sin_port = htons(PORT);        // 设置监听端口

  // 绑定套接字到指定端口
  if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    std::cerr << "绑定失败" << std::endl;
    close(sockfd);
    return -1;
  }

  std::cout << "服务器启动，监听端口 " << PORT << "...\n";

  char buffer[BUFFER_SIZE];
  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);

  // 打开文件用于写入接收到的数据
  std::ofstream outfile("gpchc.txt",
                        std::ios::app);  // 以追加模式打开文件
  if (!outfile.is_open()) {
    std::cerr << "无法打开文件" << std::endl;
    close(sockfd);
    return -1;
  }

  // 持续接收数据
  while (true) {
    // 接收客户端数据
    ssize_t recv_len = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                                (struct sockaddr*)&client_addr, &addr_len);
    if (recv_len < 0) {
      std::cerr << "接收数据失败" << std::endl;
      continue;
    }

    // 将接收到的数据添加终止符
    buffer[recv_len] = '\0';

    // 打印客户端发送的消息
    std::cout << "收到来自 " << inet_ntoa(client_addr.sin_addr) << ":"
              << ntohs(client_addr.sin_port) << " 的消息: " << buffer
              << std::endl;

    // 将接收到的数据写入文件
    outfile << buffer << std::endl;
  }

  outfile.close();
  // 关闭套接字
  close(sockfd);
  return 0;
}
