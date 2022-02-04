#include "cmd_mux.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<romea::CmdMux>();
  node->onInit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
