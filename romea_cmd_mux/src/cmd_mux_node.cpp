#include <rclcpp/rclcpp.hpp>
#include "romea_cmd_mux/cmd_mux.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::SingleThreadedExecutor exec;
  romea::CmdMux cmd_mux(options);
  exec.add_node(cmd_mux.get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

