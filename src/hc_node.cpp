#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fstream>

using namespace std;

#include "GNSS_NMEAMessage.hpp"
#include "GNSS_NovatelMessage.hpp"
#include "Uart.hpp"
#include "siasunLog.h"


// void CopyGpsMsg(sensor_msgs::NavSatFix &gps_msg, HuaCeGps &hcGps);

sensor_msgs::NavSatFix rtk_data;

void Decode(string &RecvBuffer)
{
    //解析NMEA
    static string NMEABuffer;
    NMEABuffer += RecvBuffer;
    while (true)
    {
        GNSS_NMEAMessage nmeaMessage(NMEABuffer);
        NMEABuffer = nmeaMessage.Message_Remained;
        if (!nmeaMessage.isSuccess())
            break;
        //GGA
        if (nmeaMessage.MessageID == "GPGGA")
        {
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

        //YBM  0412
         else if (nmeaMessage.MessageID == "GPYBM")
         {
             GNSS_GPYBMMessage ybm(nmeaMessage);
             printf("Recv ybm Message:\r\n");
             printf("SN                %s\r\n", ybm.SN.c_str());
             printf("utc               %f\r\n", ybm.utc);
             printf("latitude_D        %f\r\n", ybm.latitude_D); // 北纬
             printf("longitude_D       %f\r\n", ybm.longitude_D); // 东经
             printf("altitude          %f\r\n", ybm.altitude); // 海拔
             printf("pow               %f\r\n", ybm.pow); // 与正北方向夹角
             printf("pitch             %f\r\n", ybm.pitch); // 俯仰角
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
            //  rtk_data.header.frame_id = "map";
            //  rtk_data.header.stamp = ros::Time::now();
            //  rtk_data.latitude = ybm.latitude_D;
            //  rtk_data.longitude = ybm.longitude_D;
            //  rtk_data.altitude = ybm.altitude;
            //  rtk_data.status.status = ybm.fix_status;
            //  rtk_data.position_covariance[0] = ybm.pow;
         }
        // GPCHC
        else if (nmeaMessage.MessageID == "GPCHC")
        {
            GNSS_GPCHCMessage chc(nmeaMessage);
            printf("Recv CHC Message:\r\n");
            printf("m_iGPSWeek            %d\r\n", chc.m_iGPSWeek);
            printf("m_dGPSTime            %f\r\n", chc.m_dGPSTime);
            printf("m_dHeading            %f\r\n", chc.m_dHeading);// 与正北方向夹角
            printf("m_dPitch              %f\r\n", chc.m_dPitch);
            printf("m_dRoll               %f\r\n", chc.m_dRoll);
            printf("m_dgyrox              %f\r\n", chc.m_dgyrox); 
            printf("m_dgyroy              %f\r\n", chc.m_dgyroy); 
            printf("m_dgyroz              %f\r\n", chc.m_dgyroz);
            printf("m_daccx               %f\r\n", chc.m_daccx);
            printf("m_daccy               %f\r\n", chc.m_daccy);
            printf("m_daccz               %f\r\n", chc.m_daccz);
            printf("m_dLattitude          %f\r\n", chc.m_dLattitude); // 北纬
            printf("m_dLongitude          %f\r\n", chc.m_dLongitude); // 东经
            printf("m_dAltitude           %f\r\n", chc.m_dAltitude); // 海拔
            printf("m_dVe                 %f\r\n", chc.m_dVe);
            printf("m_dVn                 %f\r\n", chc.m_dVn);
            printf("m_dVu                 %f\r\n", chc.m_dVu);
            printf("m_dV                  %f\r\n", chc.m_dV);
            printf("m_iNSV1               %d\r\n", chc.m_iNSV1);
            printf("m_iNSV2               %d\r\n", chc.m_iNSV2);
            printf("m_iStatus             %d\r\n", chc.m_iStatus);
            printf("m_iAge                %d\r\n\r\n\r\n", chc.m_iAge); 
            rtk_data.header.frame_id = "map";    
            rtk_data.header.stamp = ros::Time::now();
            rtk_data.latitude = chc.m_dLattitude;
            rtk_data.longitude = chc.m_dLongitude;
            rtk_data.altitude = chc.m_dAltitude;
            rtk_data.status.status = chc.m_iStatus;
            rtk_data.position_covariance[0] = chc.m_dHeading;

        }
        //other
        else
        {
            // printf("This is unknown Message: %s\n", nmeaMessage.MessageID.c_str());
        }
    }
#if 1
    //解析Novatel
    static string NovatelBuffer;
    NovatelBuffer += RecvBuffer;
    while (true)
    {
        GNSS_NovatelMessage novatelMessage(NovatelBuffer);
        NovatelBuffer = novatelMessage.Message_Remained;
        if (!novatelMessage.isSuccess())
            break;

        //41
        if (novatelMessage.MessageID == 42)
        {
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
        //other
        else
        {
            printf("this is unknown Message: %d", novatelMessage.MessageID);
        }
    }
#endif
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hc_driver_node");
    ros::NodeHandle nh;

    static const int PORT = 9902;

    int serverSocket, clientSocket;
    struct sockaddr_in serverAddress, clientAddress;
    socklen_t clientLength;

    // 创建socket
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0) {
        std::cerr << "Failed to create socket." << std::endl;
        return 1;
    }

    // 设置服务器地址
    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(PORT);

    // 绑定socket到指定地址和端口
    if (bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Failed to bind socket." << std::endl;
        return 1;
    }

    // 监听连接
    if (listen(serverSocket, 5) < 0) {
        std::cerr << "Failed to listen." << std::endl;
        return 1;
    }

    std::cout << "Server listening on port 1234..." << std::endl;
    char buffer[1024];

    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/hc_driver/gps_data", 10);

    std::ofstream outputFile;
    outputFile.open("data.txt");  // 默认模式，不指定追加模式

    while (ros::ok())
    {
        clientLength = sizeof(clientAddress);
        clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddress, &clientLength);
        if (clientSocket < 0) {
            std::cerr << "Failed to accept connection." << std::endl;
            continue;
        }

        // 从客户端接收数据
        ssize_t bytesRead;
        while ((bytesRead = read(clientSocket, buffer, sizeof(buffer))) > 0 && ros::ok()) {
            std::cout << "Received data: " << std::string(buffer, bytesRead) << std::endl;
            // 在这里可以对接收到的数据进行处理

            std::string receiveBuffer = buffer;

            outputFile << receiveBuffer << std::endl;

            Decode(receiveBuffer);

            gps_pub.publish(rtk_data);
            // 清空缓冲区
            memset(buffer, 0, sizeof(buffer));
            ros::spinOnce();
        }

        if (bytesRead == 0) {
        // 连接关闭
        std::cout << "Client disconnected." << std::endl;
        } else {
        // 读取数据时发生错误
        std::cerr << "Failed to receive data." << std::endl;
        }

        close(clientSocket);
        // std::cout << "Received data: " << hcGps.GetBuffer() << std::endl;
    }
    close(serverSocket);
    outputFile.close();

    return 0;
}