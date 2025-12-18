#include "LIVMapper.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("laserMapping", options);

  auto mapper = std::make_shared<LIVMapper>(node, "laserMapping", options);
  mapper->InitializeSubscribersAndPublishers();
  mapper->Run();

  rclcpp::shutdown();
  return 0;
}