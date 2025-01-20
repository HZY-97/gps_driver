/**
 * @file json_pub_sub.cpp
 * @author huizeyu (huizeyu@siasun.com)
 * @brief
 * @version 0.1
 * @date 2024-12-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <atomic>
#include <cstdint>
#include <iostream>
// #include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "ddstool.hpp"
#include "json.hpp"

std::atomic_bool is_run;

void subCB(const std::string& info, const std::vector<char>& data);
int count = 0;

int main(int argc, char** argv) {
  // rclcpp::InitOptions rcl_options;
  // rcl_options.auto_initialize_logging(false);
  // rclcpp::init(argc, argv);
  is_run.store(true);

  std::shared_ptr<DDSTool> dds_ptr = std::make_shared<DDSTool>();
  std::string group = "test";
  std::string topic_web = "web_data";
  dds_ptr->load();  // 初始化dds
  bool a = dds_ptr->createSub(group, topic_web, subCB);

  while (is_run.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  dds_ptr->unload();
}

void subCB(const std::string& info, const std::vector<char>& data) {
  std::cout << "subCB: " << info << std::endl;

  try {
    // 将字符缓冲区转为字符串
    std::string json_str(data.begin(), data.end());

    // 解析 JSON 字符串
    nlohmann::json json_data = nlohmann::json::parse(json_str);

    // 打印格式化 JSON
    std::cout << json_data.dump(4) << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "JSON parse error: " << e.what() << std::endl;
  }
}