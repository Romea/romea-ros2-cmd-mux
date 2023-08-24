// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
  template<typename Node>
  ROMEA_CMD_MUX_UTILS_PUBLIC
  explicit CmdMuxUnsubscriptionClient(std::shared_ptr<Node> node);

  ROMEA_CMD_MUX_UTILS_PUBLIC
  void unsubscribe(const std::string & topic);

private:
  Result unsubscribe_(const std::string & topic);

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  const rclcpp::Logger & logger_;
  rclcpp::Client<Service>::SharedPtr client_;
};

//-----------------------------------------------------------------------------
template<typename Node>
CmdMuxUnsubscriptionClient::CmdMuxUnsubscriptionClient(std::shared_ptr<Node> node)
: node_(node->get_node_base_interface()), logger_(node->get_logger()), client_(nullptr)
{
  using ServiceType = romea_cmd_mux_msgs::srv::Unsubscribe;
  client_ = node->template create_client<ServiceType>("cmd_mux/unsubscribe");
}

}  // namespace romea

#endif  // ROMEA_CMD_MUX_UTILS__CMD_MUX_UNSUBSCRIPTION_CLIENT_HPP_
