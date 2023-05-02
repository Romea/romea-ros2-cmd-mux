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


#ifndef ROMEA_CMD_MUX__SUBSCRIBER_HPP_
#define ROMEA_CMD_MUX__SUBSCRIBER_HPP_

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"

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
