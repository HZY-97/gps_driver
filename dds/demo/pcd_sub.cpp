/**
 * @file pcd_sub.cpp
 * @author huizeyu (huizeyu@siasun.com)
 * @brief
 * @version 0.1
 * @date 2024-12-03
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

#include "ddstool.hpp"
std::atomic_bool is_run;

void subCB(const std::string& info, const std::vector<char>& data);
pcl::PointCloud<pcl::PointXYZI>::Ptr deserializePointCloud(
    const std::vector<char>& buffer);
int count = 0;

int main(int argc, char** argv) {
  is_run.store(true);

  std::shared_ptr<DDSTool> dds_ptr = std::make_shared<DDSTool>();
  std::string group = "test";
  std::string topic_web = "pcd_data";
  dds_ptr->load();  // 初始化dds
  bool a = dds_ptr->createSub(group, topic_web, subCB);

  while (is_run.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  dds_ptr->unload();
}

void subCB(const std::string& info, const std::vector<char>& data) {
  std::cout << "subCB: " << info << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = deserializePointCloud(data);

  std::cout << "cloud size: " << cloud->size() << std::endl;
  std::cout << std::endl;
}

// 反序列化函数：将 vector<char> 转换为点云数据
pcl::PointCloud<pcl::PointXYZI>::Ptr deserializePointCloud(
    const std::vector<char>& buffer) {
  // 创建点云对象
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  // 读取点的数量
  size_t num_points;
  std::memcpy(&num_points, buffer.data(), sizeof(size_t));

  // 读取点数据
  cloud->points.resize(num_points);  // 分配点数据空间
  std::memcpy(cloud->points.data(), buffer.data() + sizeof(size_t),
              num_points * sizeof(pcl::PointXYZI));

  return cloud;
}
