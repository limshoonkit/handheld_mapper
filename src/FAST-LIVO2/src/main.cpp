#include "LIVMapper.h"
#include "glogger.h"
#include <rclcpp/rclcpp.hpp>

DEFINE_string(camera_config, "config/camera_fisheye_HILTI22.yaml", "camera config file.");

int main(int argc, char **argv) {
  // 方便报错时找到出错的位置
  GLogger glogger(argc, argv, "", "");
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("laserMapping");
  rclcpp::NodeOptions options;

  auto mapper = std::make_shared<LIVMapper>(node, "laserMapping", options, FLAGS_camera_config);
  mapper->InitializeSubscribersAndPublishers();
  mapper->Run();

  rclcpp::shutdown();
  return 0;
}