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
#include <utility>

// local
#include "romea_cmd_mux/cmd_mux.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
CmdMux::CmdMux(const rclcpp::NodeOptions & options)
: mutex_(),
  node_(std::make_shared<rclcpp::Node>("cmd_mux", options)),
  publisher_(),
  subscribers_(),
  subscribe_service_(),
  unsubscribe_service_(),
  topics_type_(),
  diagnostic_msg_(),
  diagnostic_publisher_(),
  timer_()
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  node_->declare_parameter<std::string>("topics_type");

  if (!node_->get_parameter("topics_type", topics_type_)) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(), "Parameter topics_type is not defined, cannot initialize cmd_mux");
    return;
  }

  subscribe_service_ = node_->create_service<romea_cmd_mux_msgs::srv::Subscribe>(
    "~/subscribe", std::bind(&CmdMux::subscribe_callback_, this, _1, _2));

  unsubscribe_service_ = node_->create_service<romea_cmd_mux_msgs::srv::Unsubscribe>(
    "~/unsubscribe", std::bind(&CmdMux::unsubscribe_callback_, this, _1, _2));


  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  publisher_ = node_->create_generic_publisher("~/out", topics_type_, qos);

  diagnostic_msg_.status.push_back(diagnostic_msgs::msg::DiagnosticStatus());
  diagnostic_msg_.status[0].name = node_->get_fully_qualified_name();

  diagnostic_publisher_ = node_->create_publisher<DiagnosticMsg>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());

  timer_ = node_->create_wall_timer(
    std::chrono::seconds(1), std::bind(&CmdMux::timer_callback_, this));

  // status_.
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
CmdMux::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void CmdMux::subscribe_callback_(
  const SubscribeRequestSharedPtr request,
  SubscribeResponseSharedPtr response)
{
  using std::placeholders::_1;
  std::lock_guard<std::mutex> lock(mutex_);
  using Response = romea_cmd_mux_msgs::srv::Subscribe::Response;

  auto lambda = [&request](const std::pair<unsigned char, Subscriber> & s)
    {
      return s.second.sub->get_topic_name() == request->topic;
    };


  auto itTopic = std::find_if(
    std::cbegin(subscribers_),
    std::cend(subscribers_),
    lambda);

  if (itTopic != std::cend(subscribers_)) {
    response->result = Response::REJECTED_TOPIC_ALREADY_SUBSCRIBED;
    return;
  }


  auto itPriority = subscribers_.find(request->priority);

  if (itPriority != std::cend(subscribers_)) {
    response->result = Response::REJECTED_PRIORITY_ALREADY_USED;
    return;
  }

  auto & subscriber = subscribers_[request->priority];

  subscriber.timeout = rclcpp::Duration::from_seconds(request->timeout);
  subscriber.msg_stamp = rclcpp::Time(0ULL, node_->get_clock()->get_clock_type());

  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> f =
    std::bind(&CmdMux::publish_callback_, this, _1, request->priority);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  rclcpp::SubscriptionOptions options;
  // TODO(JEAN) search why MutuallyExclusive callback group don't work
  // options.callback_group =
  //   node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  subscriber.sub =
    node_->create_generic_subscription(request->topic, topics_type_, qos, f, options);

  response->result = Response::ACCEPTED;
}

//-----------------------------------------------------------------------------
void CmdMux::unsubscribe_callback_(
  const UnsubscribeRequestSharedPtr request,
  UnsubscribeResponseSharedPtr response)
{
  std::lock_guard<std::mutex> lock(mutex_);


  auto lambda = [&](const std::pair<unsigned char, Subscriber> & s)
    {
      return s.second.sub->get_topic_name() == request->topic;
    };

  auto it = std::find_if(
    std::cbegin(subscribers_),
    std::cend(subscribers_),
    lambda);

  if (it != subscribers_.end()) {
    subscribers_.erase(it);
    response->result = romea_cmd_mux_msgs::srv::Unsubscribe::Response::ACCEPTED;
  } else {
    response->result = romea_cmd_mux_msgs::srv::Unsubscribe::Response::REJECTED;
  }
}


//-----------------------------------------------------------------------------
void CmdMux::publish_callback_(MsgSharedPtr msg, unsigned char priotity)
{
  std::lock_guard<std::mutex> lock(mutex_);

  rclcpp::Time now = node_->get_clock()->now();
  auto it = subscribers_.find(priotity);
  (*it).second.msg_stamp = now;

  if (has_highest_priority_(it, now)) {
    publisher_->publish(*msg);
  }
}

//-----------------------------------------------------------------------------
bool CmdMux::has_highest_priority_(SubscriberMap::iterator it, const rclcpp::Time & now)
{
  while (++it != subscribers_.end()) {
    if (now - (*it).second.msg_stamp < (*it).second.timeout) {
      return false;
    }
  }
  return true;
}

//-----------------------------------------------------------------------------
void CmdMux::timer_callback_()
{
  rclcpp::Time now = node_->get_clock()->now();
  diagnostic_msg_.status[0].values.clear();
  diagnostic_msg_.header.stamp = now;

  std::lock_guard<std::mutex> lock(mutex_);

  if (subscribers_.empty()) {
    diagnostic_msg_.status[0].message = "No node is connected to mutiplexer";
    diagnostic_msg_.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else {
    diagnostic_msg_.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  for (const auto &[priority, subscriber] : subscribers_) {
    diagnostic_msgs::msg::KeyValue key;
    key.key = std::string(subscriber.sub->get_topic_name());
    key.value = "priority: " + std::to_string(static_cast<int>(priority)) + ", ";
    if (now - subscriber.msg_stamp < subscriber.timeout) {
      key.value += "active";
    } else {
      key.value += "inactive";
    }
    diagnostic_msg_.status[0].values.push_back(key);
  }
  diagnostic_publisher_->publish(diagnostic_msg_);
}

}  /// namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::CmdMux)
