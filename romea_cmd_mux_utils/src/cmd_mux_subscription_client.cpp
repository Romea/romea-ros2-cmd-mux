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
#include "romea_cmd_mux_utils/cmd_mux_subscription_client.hpp"

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
void log_subscription_has_been_accepted_(
  const rclcpp::Logger & logger, const std::string & service_name, const std::string & topic)
{
  std::string cmd_mux_name = extract_cmd_mux_name(service_name);

  std::stringstream msg;
  msg << "Subscription request for topic " << topic;
  msg << " has been accepted by " + cmd_mux_name;
  RCLCPP_INFO_STREAM(logger, msg.str());
}

//-----------------------------------------------------------------------------
void throw_topic_is_already_registered(const std::string & service_name, const std::string & topic)
{
  std::string cmd_mux_name = extract_cmd_mux_name(service_name);

  std::stringstream msg;
  msg << " Subscription request for topic " << topic;
  msg << " has been rejected by " + cmd_mux_name;
  msg << " because topic " << topic << " is already suscribed ";
  throw std::runtime_error(msg.str());
}

//-----------------------------------------------------------------------------
void throw_priority_is_already_used(const std::string & service_name, const std::string & topic)
{
  std::string cmd_mux_name = extract_cmd_mux_name(service_name);

  std::stringstream msg;
  msg << " Subscription request for topic " << topic;
  msg << " has been rejected by " + cmd_mux_name;
  msg << " because an another topic with the same priority is already suscribed";
  throw std::runtime_error(msg.str());
}

//-----------------------------------------------------------------------------
void throw_fail_to_send_subscription_request(
  const std::string & service_name, const std::string & topic)
{
  std::string cmd_mux_name = extract_cmd_mux_name(service_name);

  std::stringstream msg;
  msg << "Failed to send to subscription resquest to service " << service_name;
  msg << "," << cmd_mux_name << " will not subscribe to topic " << topic;
  throw std::runtime_error(msg.str());
}

//-----------------------------------------------------------------------------
void throw_fail_to_call_subscription_service(
  const std::string & service_name, const std::string & topic)
{
  std::string cmd_mux_name = extract_cmd_mux_name(service_name);

  std::stringstream msg;
  msg << "Failed to call service " << service_name;
  msg << "," << cmd_mux_name << " will not subscribe to topic " << topic;
  throw std::runtime_error(msg.str());
}

}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
CmdMuxSubscriptionClient::Result CmdMuxSubscriptionClient::subscribe_(
  const std::string & topic, int priority, double timeout)
{
  if (client_->wait_for_service(WAIT_FOR_SERVICE_TIMEOUT)) {
    using RequestType = romea_cmd_mux_msgs::srv::Subscribe::Request;
    auto request = std::make_shared<RequestType>();
    request->topic = topic;
    request->priority = priority;
    request->timeout = timeout;

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
      return static_cast<Result>(result.get()->result);
    } else {
      return FAIL_TO_SEND_REQUEST;
    }
  } else {
    return FAIL_TO_CALL_SERVICE;
  }
}

//-----------------------------------------------------------------------------
void CmdMuxSubscriptionClient::subscribe(
  const std::string & topic_name, int priority, double timeout)
{
  std::string service_name = client_->get_service_name();

  switch (subscribe_(topic_name, priority, timeout)) {
    case Result::ACCEPTED:
      log_subscription_has_been_accepted_(logger_, service_name, topic_name);
      break;
    case Result::FAIL_TO_CALL_SERVICE:
      throw_fail_to_call_subscription_service(service_name, topic_name);
      break;
    case Result::FAIL_TO_SEND_REQUEST:
      throw_fail_to_send_subscription_request(service_name, topic_name);
      break;
    case Result::REJECTED_TOPIC_ALREADY_SUBSCRIBED:
      throw_topic_is_already_registered(service_name, topic_name);
      break;
    case CmdMuxSubscriptionClient::Result::REJECTED_PRIORITY_ALREADY_USED:
      throw_priority_is_already_used(service_name, topic_name);
      break;
  }
}

}  // namespace romea
