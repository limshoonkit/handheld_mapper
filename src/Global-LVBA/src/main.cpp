#include <rclcpp/rclcpp.hpp>
#include "lvba_system.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lv_ba");
    lvba::LvbaSystem lvba_system(node);
    lvba_system.runFullPipeline();
    rclcpp::shutdown();
    return 0;
}
