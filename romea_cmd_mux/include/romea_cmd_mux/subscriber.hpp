// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_CMD_MUX__SUBSCRIBER_HPP_
#define ROMEA_CMD_MUX__SUBSCRIBER_HPP_

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>

// #include <diagnostic_updater/diagnostic_updater.h>

namespace romea
{


struct Subscriber
{
  Subscriber()
  : sub(),
    timeout(rclcpp::Duration::from_nanoseconds(0)),
    msg_stamp(0.)
  {
  }

  rclcpp::GenericSubscription::SharedPtr sub;
  rclcpp::Duration timeout;
  rclcpp::Time msg_stamp;
};


}  // namespace romea

#endif  // ROMEA_CMD_MUX__SUBSCRIBER_HPP_
