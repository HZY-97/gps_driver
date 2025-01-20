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
#include <iostream>
#include <thread>

#include "ddstool.hpp"
#include "json.hpp"

std::atomic_bool is_run;

void pubData(const std::shared_ptr<DDSTool>& dds_ptr, const std::string& topic);
int count = 0;

int main(int argc, char** argv) {
  std::thread pub_thread;

  is_run.store(true);

  std::shared_ptr<DDSTool> dds_ptr = std::make_shared<DDSTool>();
  std::string group = "test";
  std::string topic_web = "web_data";
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
  nlohmann::json pub_j;
  pub_j["command"] = "Command";
  pub_j["success"] = true;
  pub_j["data"]["x"] = 20.35;
  pub_j["data"]["y"] = -0.547;
  pub_j["data"]["z"] = -7.5;

  // 将 JSON 转为字符串
  std::string json_str = pub_j.dump();

  // 将字符串转为字符缓冲区
  std::vector<char> buffer(json_str.begin(), json_str.end());

  // 发送数据
  bool ret = dds_ptr->pub(topic, std::to_string(count++), buffer);
  std::cout << "pub ret: " << ret << " " << count << " times" << std::endl;
}