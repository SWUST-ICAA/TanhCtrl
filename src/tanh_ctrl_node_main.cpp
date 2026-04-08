#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "tanh_ctrl/tanh_ctrl_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tanh_ctrl::tanh_ctrl_node>());
  rclcpp::shutdown();
  return 0;
}
