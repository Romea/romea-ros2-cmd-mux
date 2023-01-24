// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CMD_MUX_UTILS__CMD_MUX_UNSUBSCRIPTION_CLIENT_HPP_
#define ROMEA_CMD_MUX_UTILS__CMD_MUX_UNSUBSCRIPTION_CLIENT_HPP_


// std
#include <memory>
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"

// romea ros
#include "romea_cmd_mux_msgs/srv/unsubscribe.hpp"

// local
#include "romea_cmd_mux_utils/visibility_control.h"

namespace romea
{

class CmdMuxUnsubscriptionClient
{
private:
  enum Result
  {
    ACCEPTED,
    REJECTED,
    FAIL_TO_CALL_SERVICE,
    FAIL_TO_SEND_REQUEST
  };

  using Service = romea_cmd_mux_msgs::srv::Unsubscribe;

public:
  ROMEA_CMD_MUX_UTILS_PUBLIC
  explicit CmdMuxUnsubscriptionClient(std::shared_ptr<rclcpp::Node> node);

  ROMEA_CMD_MUX_UTILS_PUBLIC
  void unsubscribe(const std::string & topic);

private:
  Result unsubscribe_(const std::string & topic);

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<Service>::SharedPtr client_;
};

}  // namespace romea

#endif  // ROMEA_CMD_MUX_UTILS__CMD_MUX_UNSUBSCRIPTION_CLIENT_HPP_
