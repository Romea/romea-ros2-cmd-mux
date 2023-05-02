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


// std
#include <memory>
#include <string>

// local
#include "romea_cmd_mux_utils/cmd_mux_unsubscription_client.hpp"

namespace
{
const std::chrono::seconds WAIT_FOR_SERVICE_TIMEOUT(5);

//-----------------------------------------------------------------------------
std::string extract_cmd_mux_name(const std::string & service_name)
{
  auto pos = service_name.find_last_of("/");
  return service_name.substr(0, pos);
}

//-----------------------------------------------------------------------------
void log_ununsubscription_has_been_accepted(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & service_name,
  const std::string & topic)
{
  std::string cmd_mux_name = extract_cmd_mux_name(service_name);

  std::stringstream msg;
  msg << "Ununsubscription request for topic " << topic;
  msg << " has been accepted by " + cmd_mux_name;
  RCLCPP_INFO_STREAM(node->get_logger(), msg.str());
}

//-----------------------------------------------------------------------------
void throw_ununsubscription_has_been_rejected(
  const std::string & service_name,
  const std::string & topic)
{
  std::string cmd_mux_name = extract_cmd_mux_name(service_name);

  std::stringstream msg;
  msg << "Ununsubscription request for topic " << topic;
  msg << " has been rejected by " + cmd_mux_name;
  throw std::runtime_error(msg.str());
}

//-----------------------------------------------------------------------------
void throw_fail_to_send_ununsubscription_request(
  const std::string & service_name,
  const std::string & topic)
{
  std::string cmd_mux_name = extract_cmd_mux_name(service_name);

  std::stringstream msg;
  msg << "Failed to send to ununsubscription resquest to service " << service_name;
  msg << ", cannot unregister topic " << topic << " from " << cmd_mux_name;
  throw std::runtime_error(msg.str());
}


//-----------------------------------------------------------------------------
void throw_fail_to_call_ununsubscription_service(
  const std::string & service_name,
  const std::string & topic)
{
  std::string cmd_mux_name = extract_cmd_mux_name(service_name);

  std::stringstream msg;
  msg << "Failed to call service " << service_name;
  msg << ", cannot unregister topic " << topic << " from " << cmd_mux_name;
  throw std::runtime_error(msg.str());
}

}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
CmdMuxUnsubscriptionClient::CmdMuxUnsubscriptionClient(std::shared_ptr<rclcpp::Node> node)
: node_(node),
  client_(nullptr)
{
  using ServiceType = romea_cmd_mux_msgs::srv::Unsubscribe;
  client_ = node->create_client<ServiceType>("cmd_mux/unsubscribe");
}


//-----------------------------------------------------------------------------
CmdMuxUnsubscriptionClient::Result
CmdMuxUnsubscriptionClient::unsubscribe_(const std::string & topic)
{
  if (client_->wait_for_service(WAIT_FOR_SERVICE_TIMEOUT)) {
    using RequestType = romea_cmd_mux_msgs::srv::Unsubscribe::Request;
    auto request = std::make_shared<RequestType>();
    request->topic = topic;

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return static_cast<Result>(result.get()->result);
    } else {
      return FAIL_TO_SEND_REQUEST;
    }
  } else {
    return FAIL_TO_CALL_SERVICE;
  }
}

//-----------------------------------------------------------------------------
void CmdMuxUnsubscriptionClient::unsubscribe(const std::string & topic_name)
{
  std::string service_name = client_->get_service_name();

  switch (unsubscribe_(topic_name)) {
    case Result::ACCEPTED:
      log_ununsubscription_has_been_accepted(node_, service_name, topic_name);
      break;
    case Result::REJECTED:
      throw_ununsubscription_has_been_rejected(service_name, topic_name);
      break;
    case Result::FAIL_TO_CALL_SERVICE:
      throw_fail_to_call_ununsubscription_service(service_name, topic_name);
      break;
    case Result::FAIL_TO_SEND_REQUEST:
      throw_fail_to_send_ununsubscription_request(service_name, topic_name);
      break;
  }
}

}   // namespace romea
