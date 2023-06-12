#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "UdpSocketReceiver.h"
#include "HuaCeGps.h"
#include "siasunLog.h"

void CopyGpsMsg(sensor_msgs::NavSatFix &gps_msg, HuaCeGps &hcGps);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hc_driver_node");
    ros::NodeHandle nh;

    static const int PORT = 9902;
    HuaCeGps hcGps(PORT);
    hcGps.InitUdp();

    sensor_msgs::NavSatFix gps_msg;

    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/hc_driver/gps_data", 10);

    while (ros::ok())
    {
        hcGps.DoReceive();
        hcGps.PrintData();
        CopyGpsMsg(gps_msg, hcGps);
        gps_pub.publish(gps_msg);
        ros::spinOnce();
        // std::cout << "Received data: " << hcGps.GetBuffer() << std::endl;
    }
}

void CopyGpsMsg(sensor_msgs::NavSatFix &gps_msg, HuaCeGps &hcGps)
{
    gps_msg.header.frame_id = "map";
    gps_msg.header.stamp = ros::Time::now();

    gps_msg.status.status = hcGps.m_iStatus;

    gps_msg.latitude = hcGps.m_dLattitude;  // 设置纬度值
    gps_msg.longitude = hcGps.m_dLongitude; // 设置经度值
    gps_msg.altitude = hcGps.m_dAltitude;   // 设置海拔值

    gps_msg.position_covariance[0] = hcGps.m_dHeading;
    // gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}