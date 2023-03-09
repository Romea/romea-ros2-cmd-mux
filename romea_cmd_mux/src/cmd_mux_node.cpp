// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>

// local
#include "romea_cmd_mux/cmd_mux.hpp"


int main(int argc, char * argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  rclcpp::NodeOptions options;
  options.arguments(args);
  auto node = std::make_shared<romea::CmdMux>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
