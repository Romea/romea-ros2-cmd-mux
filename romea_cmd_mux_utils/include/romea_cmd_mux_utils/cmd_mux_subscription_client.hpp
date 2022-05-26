#ifndef CmdMuxSubscriber_HPP
#define CmdMuxSubscriber_HPP

//ros
#include <rclcpp/rclcpp.hpp>
#include <romea_cmd_mux_msgs/srv/subscribe.hpp>

//local
#include "visibility_control.h"

namespace romea
{

class CmdMuxSubscriptionClient
{

private :

  enum Result {
    ACCEPTED,
    REJECTED_TOPIC_ALREADY_SUBSCRIBED,
    REJECTED_PRIORITY_ALREADY_USED,
    FAIL_TO_CALL_SERVICE,
    FAIL_TO_SEND_REQUEST
  };

  using Service = romea_cmd_mux_msgs::srv::Subscribe;

public:

  ROMEA_CMD_MUX_UTILS_PUBLIC
  CmdMuxSubscriptionClient(std::shared_ptr<rclcpp::Node> node);

  ROMEA_CMD_MUX_UTILS_PUBLIC
  void subscribe(const std::string & topic,
                 const int & priority,
                 const double & timeout);

private:

  Result subsribe_(const std::string & topic,
                   const int & priority,
                   const double & timeout);

private:

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Service>::SharedPtr client_;
};


}

#endif
