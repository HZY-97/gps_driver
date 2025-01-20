/**
 * @file pcd_pub.cpp
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
#include <iostream>
#include <thread>

#include "ddstool.hpp"

const std::string pcd_root_path = "/home/siasun/bag_map/map/lego_test/SurfMap/";
const int max_idx = 5742;
int current_idx = 0;
int count = 0;

std::atomic_bool is_run;

pcl::PCDReader pcd_reader;

void pubData(const std::shared_ptr<DDSTool>& dds_ptr, const std::string& topic);
std::vector<char> serializePointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

int main(int argc, char** argv) {
  std::thread pub_thread;

  is_run.store(true);

  std::shared_ptr<DDSTool> dds_ptr = std::make_shared<DDSTool>();
  std::string group = "test";
  std::string topic_web = "pcd_data";
  dds_ptr->load();  // 初始化dds
  dds_ptr->createPub(group, topic_web);
  pub_thread = std::thread([&]() {
    while (is_run.load()) {
      pubData(dds_ptr, topic_web);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  pub_thread.join();

  dds_ptr->unload();
}

void pubData(const std::shared_ptr<DDSTool>& dds_ptr,
             const std::string& topic) {
  if (current_idx > max_idx) {
    current_idx = 0;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  pcd_reader.read(pcd_root_path + std::to_string(current_idx++) + ".pcd",
                  *cloud);

  std::cout << "cloud size:" << cloud->size() << std::endl;

  std::vector<char> buffer(serializePointCloud(cloud));
  bool ret = dds_ptr->pub(topic, std::to_string(count++), buffer);
  std::cout << "pub ret: " << ret << " " << count << " times" << std::endl;
  std::cout << std::endl;

  // 发送数据
  //   bool ret = dds_ptr->pub(topic, std::to_string(count++), buffer);
  //   std::cout << "pub ret: " << ret << " " << count << " times" << std::endl;
}

// 序列化函数：将点云数据转换为 vector<char>
std::vector<char> serializePointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  // 准备序列化缓冲区
  std::vector<char> buffer;

  // 写入点云点的数量
  size_t num_points = cloud->points.size();
  buffer.resize(sizeof(size_t));  // 先为点数量分配空间
  std::memcpy(buffer.data(), &num_points, sizeof(size_t));

  // 写入点云的每个点数据
  size_t point_data_size = num_points * sizeof(pcl::PointXYZI);
  buffer.resize(buffer.size() + point_data_size);  // 扩展缓冲区
  std::memcpy(buffer.data() + sizeof(size_t), cloud->points.data(),
              point_data_size);

  return buffer;
}