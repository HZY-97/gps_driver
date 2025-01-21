#include <arpa/inet.h>
#include <glog/logging.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>

#include <builtin_interfaces/msg/time.hpp>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>

#include "GNSS_NMEAMessage.hpp"
#include "GNSS_NovatelMessage.hpp"
#include "Uart.hpp"

const int PORT = 9996;
const int BUFFER_SIZE = 1024;

using namespace std;
// void CopyGpsMsg(sensor_msgs::NavSatFix &gps_msg, HuaCeGps &hcGps);

sensor_msgs::msg::NavSatFix rtk_data;
void Decode(string &RecvBuffer);

std::atomic<bool> running(true);
std::condition_variable cv;
std::mutex cv_mtx;

void signal_handler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    running.store(false);
    cv.notify_all();
    std::cout << "gps_driver_node程序正在退出..." << std::endl;
  }
}
class GPSPublisher : public rclcpp::Node {
 public:
  GPSPublisher() : Node("gps_driver_node") {
    // 创建发布者，发布类型为 sensor_msgs/msg/NavSatFix
    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/gps_driver/gps_data", 10);
  }

  void PubGpsData(sensor_msgs::msg::NavSatFix &gps_msg) {
    if (publisher_ != nullptr) {
      if (publisher_->get_subscription_count() > 0) {
        auto now = this->get_clock()->now();
        gps_msg.header.stamp.sec = now.seconds();
        gps_msg.header.stamp.nanosec = now.nanoseconds() % 1000000000;
        publisher_->publish(gps_msg);
      }
    }
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  std::signal(SIGINT, signal_handler);

  /** @brief ros初始化*/
  rclcpp::InitOptions rcl_options;
  rcl_options.auto_initialize_logging(false);
  rclcpp::init(argc, argv, rcl_options);
  std::shared_ptr<GPSPublisher> gps_pub = std::make_shared<GPSPublisher>();
  /** @brief ros初始化*/

  /** @brief 获取环境变量 初始化日志*/
  // std::string path;
  // char *gps_driver_path = std::getenv("GPS_DRIVER_PATH");
  // if (nullptr == gps_driver_path) {
  //   std::cerr << "\033[31m"
  //             << "[ENV]:No env 'GPS_DRIVER_PATH'"
  //             << "\033[0m" << std::endl;
  //   return -1;
  // } else {
  //   path = std::string(gps_driver_path);
  // }
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logbufsecs = 0;
  FLAGS_colorlogtostderr = true;
  FLAGS_logtostderr = true;
  LOG(INFO) << "Log init.";
  /** @brief 获取环境变量 初始化日志*/

  // 创建 UDP 套接字
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    LOG(ERROR) << "无法创建套接字";
    return -1;
  }

  // 设置服务器地址结构
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;  // 监听本机的所有 IP 地址
  server_addr.sin_port = htons(PORT);        // 设置监听端口

  // 绑定套接字到指定端口
  if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    LOG(ERROR) << "绑定失败";
    close(sockfd);
    return -1;
  }

  LOG(INFO) << "服务器启动，监听端口 " << PORT << "...";

  char buffer[BUFFER_SIZE];
  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);

  std::thread t([&]() {
    while (running.load()) {
      ssize_t bytesRead = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                                   (struct sockaddr *)&client_addr, &addr_len);
      if (bytesRead <= 0) {
        LOG(ERROR) << "Error receiving data or connection closed by the "
                      "GPS sensor.";
        continue;
        // break;
      }

      std::string receiveStr(buffer);
      LOG(INFO) << "Received data from GPS sensor:" << receiveStr;
      // outputFile << receiveStr << std::endl;

      Decode(receiveStr);
      gps_pub->PubGpsData(rtk_data);
      // 清空缓冲区
      memset(buffer, 0, sizeof(buffer));
    }
  });

  rclcpp::spin(std::make_shared<GPSPublisher>());

  std::unique_lock<std::mutex> lock(cv_mtx);
  cv.wait(lock, [] {
    return !running.load();
  });
  t.join();

  // close(clientSocket);  // 关闭GPS传感器套接字，结束与GPS传感器的连接
  close(sockfd);  // 关闭服务器套接字
  // outputFile.close();
  rclcpp::shutdown();
  return 0;
}

void Decode(string &RecvBuffer) {
  // 解析NMEA
  static string NMEABuffer;
  NMEABuffer += RecvBuffer;
  while (true) {
    GNSS_NMEAMessage nmeaMessage(NMEABuffer);
    NMEABuffer = nmeaMessage.Message_Remained;
    if (!nmeaMessage.isSuccess())
      break;
    // GGA
    if (nmeaMessage.MessageID == "GPGGA") {
      GNSS_GPGGAMessage gga(nmeaMessage);
      printf("Recv gga Message:\r\n");
      printf("utc           %f\r\n", gga.utc);
      printf("latitude      %f\r\n", gga.latitude_Dm);
      printf("latdir        %c\r\n", gga.latdir);
      printf("longitude     %f\r\n", gga.longitude_Dm);
      printf("londir        %c\r\n", gga.londir);
      printf("fix_status    %d\r\n", gga.fix_status);
      printf("fix_satellite %d\r\n", gga.fix_satellite);
      printf("hdop          %f\r\n", gga.hdop);
      printf("altitude      %f\r\n", gga.altitude);
      printf("alt_unit      %c\r\n", gga.alt_unit);
      printf("undulation    %f\r\n", gga.undulation);
      printf("und_unit      %c\r\n", gga.und_unit);
      printf("age           %d\r\n\r\n\r\n", gga.age);
      // rtk_data.header.frame_id = "map";
      // rtk_data.header.stamp = ros::Time::now();
      // rtk_data.header.frame_id = rtk;
      // rtk_data.latitude = gga.latitude_Dm;
      // rtk_data.longitude = gga.longitude_Dm;
      // rtk_data.altitude = gga.altitude;
    }

    // YBM  0412
    else if (nmeaMessage.MessageID == "GPYBM") {
      GNSS_GPYBMMessage ybm(nmeaMessage);
      printf("Recv ybm Message:\r\n");
      printf("SN                %s\r\n", ybm.SN.c_str());
      printf("utc               %f\r\n", ybm.utc);
      printf("latitude_D        %f\r\n", ybm.latitude_D);   // 北纬
      printf("longitude_D       %f\r\n", ybm.longitude_D);  // 东经
      printf("altitude          %f\r\n", ybm.altitude);     // 海拔
      printf("pow               %f\r\n", ybm.pow);    // 与正北方向夹角
      printf("pitch             %f\r\n", ybm.pitch);  // 俯仰角
      printf("speedX            %f\r\n", ybm.speedX);
      printf("speedY            %f\r\n", ybm.speedY);
      printf("speedZ            %f\r\n", ybm.speedZ);
      printf("groundspeed       %f\r\n", ybm.groundspeed);
      printf("GaussX            %f\r\n", ybm.GaussX);
      printf("GaussY            %f\r\n", ybm.GaussY);
      printf("baseX             %f\r\n", ybm.baseX);
      printf("baseY             %f\r\n", ybm.baseY);
      printf("fix_status        %d\r\n", ybm.fix_status);
      printf("fix_status2       %d\r\n", ybm.fix_status2);
      printf("fix_satellite     %d\r\n", ybm.fix_satellite);
      printf("delay             %d\r\n", ybm.delay);
      printf("BaseDist          %d\r\n", ybm.BaseDist);
      printf("Slave_fix_status  %d\r\n", ybm.Slave_fix_status);
      printf("roll              %f\r\n\r\n\r\n", ybm.roll);
      // rtk_data.header.frame_id = "map";
      // rtk_data.header.stamp = ros::Time::now();
      // rtk_data.latitude = ybm.latitude_D;
      // rtk_data.longitude = ybm.longitude_D;
      // rtk_data.altitude = ybm.altitude;
      // rtk_data.status.status = ybm.fix_status;
      // rtk_data.position_covariance[0] = ybm.pow;
    }
    // GPCHC
    else if (nmeaMessage.MessageID == "GPCHC") {
      GNSS_GPCHCMessage chc(nmeaMessage);
      printf("Recv CHC Message:\r\n");
      printf("m_iGPSWeek            %d\r\n", chc.m_iGPSWeek);
      printf("m_dGPSTime            %f\r\n", chc.m_dGPSTime);
      printf("m_dHeading            %f\r\n", chc.m_dHeading);  // 与正北方向夹角
      printf("m_dPitch              %f\r\n", chc.m_dPitch);
      printf("m_dRoll               %f\r\n", chc.m_dRoll);
      printf("m_dgyrox              %f\r\n", chc.m_dgyrox);
      printf("m_dgyroy              %f\r\n", chc.m_dgyroy);
      printf("m_dgyroz              %f\r\n", chc.m_dgyroz);
      printf("m_daccx               %f\r\n", chc.m_daccx);
      printf("m_daccy               %f\r\n", chc.m_daccy);
      printf("m_daccz               %f\r\n", chc.m_daccz);
      printf("m_dLattitude          %f\r\n", chc.m_dLattitude);  // 北纬
      printf("m_dLongitude          %f\r\n", chc.m_dLongitude);  // 东经
      printf("m_dAltitude           %f\r\n", chc.m_dAltitude);   // 海拔
      printf("m_dVe                 %f\r\n", chc.m_dVe);
      printf("m_dVn                 %f\r\n", chc.m_dVn);
      printf("m_dVu                 %f\r\n", chc.m_dVu);
      printf("m_dV                  %f\r\n", chc.m_dV);
      printf("m_iNSV1               %d\r\n", chc.m_iNSV1);
      printf("m_iNSV2               %d\r\n", chc.m_iNSV2);
      printf("m_iStatus             %d\r\n", chc.m_iStatus);
      printf("m_iAge                %d\r\n\r\n\r\n", chc.m_iAge);
      rtk_data.header.frame_id = "map";
      // rtk_data.header.stamp = ros::Time::now();
      rtk_data.latitude = chc.m_dLattitude;
      rtk_data.longitude = chc.m_dLongitude;
      rtk_data.altitude = chc.m_dAltitude;
      rtk_data.status.status = chc.m_iStatus;
      rtk_data.position_covariance[0] = chc.m_dHeading;
    }
    // other
    else {
      // printf("This is unknown Message: %s\n", nmeaMessage.MessageID.c_str());
    }
  }
#if 1
  // 解析Novatel
  static string NovatelBuffer;
  NovatelBuffer += RecvBuffer;
  while (true) {
    GNSS_NovatelMessage novatelMessage(NovatelBuffer);
    NovatelBuffer = novatelMessage.Message_Remained;
    if (!novatelMessage.isSuccess())
      break;

    // 41
    if (novatelMessage.MessageID == 42) {
      GNSS_BESTPOSB bestpos(novatelMessage);
      printf("Recv Bestpos Message:\r\n");
      printf("uGpsWeek        %d\r\n", bestpos.uGpsWeek);
      printf("fGpsWeekSecond  %f\r\n", bestpos.fGpsWeekSecond);
      printf("sol_stat        %d\r\n", bestpos.sol_stat);
      printf("pos_type        %d\r\n", bestpos.pos_type);
      printf("lat             %f\r\n", bestpos.lat);
      printf("lon             %f\r\n", bestpos.lon);
      printf("hgt             %f\r\n", bestpos.hgt);
      printf("SVs             %d\r\n", bestpos.SVs);
      printf("sonlnSVs        %d\r\n\r\n\r\n", bestpos.sonlnSVs);
    }
    // other
    else {
      printf("this is unknown Message: %d", novatelMessage.MessageID);
    }
  }
#endif
}
