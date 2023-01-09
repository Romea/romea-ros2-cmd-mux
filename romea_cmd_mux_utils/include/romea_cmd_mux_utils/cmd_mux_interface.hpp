// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CMD_MUX_UTILS__CMD_MUX_INTERFACE_HPP_
#define ROMEA_CMD_MUX_UTILS__CMD_MUX_INTERFACE_HPP_

// std
#include <memory>
#include <list>
#include <string>

// romea
#include "romea_cmd_mux_utils/cmd_mux_subscription_client.hpp"
#include "romea_cmd_mux_utils/cmd_mux_unsubscription_client.hpp"

namespace romea
{

class CmdMuxInterface
{
public:
  ROMEA_CMD_MUX_UTILS_PUBLIC
  explicit CmdMuxInterface(std::shared_ptr<rclcpp::Node> node);

  ROMEA_CMD_MUX_UTILS_PUBLIC
  ~CmdMuxInterface() = default;

  ROMEA_CMD_MUX_UTILS_PUBLIC
  void subscribe(
    const std::string & topic,
    const int & priority,
    const double & timeout);

  ROMEA_CMD_MUX_UTILS_PUBLIC
  void unsubscribe(const std::string & topic);

private:
  CmdMuxSubscriptionClient subscription_;
  CmdMuxUnsubscriptionClient unsubscription_;
  std::list<std::string> subscribed_topics_;
};

}  // namespace romea

#endif  // ROMEA_CMD_MUX_UTILS__CMD_MUX_INTERFACE_HPP_
