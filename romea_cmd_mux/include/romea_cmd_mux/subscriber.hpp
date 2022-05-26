#ifndef Subscriber_HPP
#define Subscriber_HPP

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
//#include <diagnostic_updater/diagnostic_updater.h>

namespace romea
{


struct Subscriber
{
  Subscriber():
    sub(),
    timeout(rclcpp::Duration::from_nanoseconds(0)),
    msg_stamp(0.)
  {

  }

  rclcpp::GenericSubscription::SharedPtr sub;
  rclcpp::Duration timeout;
  rclcpp::Time msg_stamp;
};


}

#endif
