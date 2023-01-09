// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CMD_MUX__CMD_MUX_HPP_
#define ROMEA_CMD_MUX__CMD_MUX_HPP_


// ros
#include <romea_cmd_mux_msgs/srv/subscribe.hpp>
#include <romea_cmd_mux_msgs/srv/unsubscribe.hpp>

// std
#include <map>
#include <memory>
#include <mutex>
#include <string>

// local
#include "romea_cmd_mux/subscriber.hpp"
#include "romea_cmd_mux/visibility_control.h"


namespace romea
{

class CmdMux
{
protected:
  using SubscriberMap = std::map<unsigned char, Subscriber>;
  using PublisherSharedPtr = rclcpp::GenericPublisher::SharedPtr;
  using MsgSharedPtr = std::shared_ptr<rclcpp::SerializedMessage>;

  using SubscribeServiceSharedPtr =
    rclcpp::Service<romea_cmd_mux_msgs::srv::Subscribe>::SharedPtr;
  using SubscribeRequestSharedPtr =
    std::shared_ptr<romea_cmd_mux_msgs::srv::Subscribe::Request>;
  using SubscribeResponseSharedPtr =
    std::shared_ptr<romea_cmd_mux_msgs::srv::Subscribe::Response>;
  using UnsubscribeServiceSharedPtr =
    rclcpp::Service<romea_cmd_mux_msgs::srv::Unsubscribe>::SharedPtr;
  using UnsubscribeRequestSharedPtr =
    std::shared_ptr<romea_cmd_mux_msgs::srv::Unsubscribe::Request>;
  using UnsubscribeResponseSharedPtr =
    std::shared_ptr<romea_cmd_mux_msgs::srv::Unsubscribe::Response>;

public:
  ROMEA_CMD_MUX_PUBLIC
  explicit CmdMux(const rclcpp::NodeOptions & options);

  ROMEA_CMD_MUX_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:
  bool has_highest_priority_(
    SubscriberMap::iterator it,
    const rclcpp::Time & now);

  void subscribe_callback_(
    const SubscribeRequestSharedPtr request,
    SubscribeResponseSharedPtr response);

  void unsubscribe_callback_(
    const UnsubscribeRequestSharedPtr request,
    UnsubscribeResponseSharedPtr response);

  void publish_callback_(MsgSharedPtr msg, unsigned char priotity);

protected:
  std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  PublisherSharedPtr publisher_;
  SubscriberMap subscribers_;

  SubscribeServiceSharedPtr subscribe_service_;
  UnsubscribeServiceSharedPtr unsubscribe_service_;
  std::string topics_type_;
};

}  // namespace romea

#endif  // ROMEA_CMD_MUX__CMD_MUX_HPP_
