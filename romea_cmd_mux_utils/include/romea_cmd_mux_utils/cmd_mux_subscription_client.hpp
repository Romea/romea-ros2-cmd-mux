#ifndef CMD_MUX_UTILS_CMD_MUX_SUBSCRIPTION_CLIENT_HPP_
#define CMD_MUX_UTILS_CMD_MUX_SUBSCRIPTION_CLIENT_HPP_

// std
#include <memory>
#include <string>

// ros
#include <rclcpp/rclcpp.hpp>
#include <romea_cmd_mux_msgs/srv/subscribe.hpp>

// local
#include "romea_cmd_mux_utils/visibility_control.h"

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
  explicit CmdMuxSubscriptionClient(std::shared_ptr<rclcpp::Node> node);

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

}  // namespace romea

#endif  // CMD_MUX_UTILS_CMD_MUX_SUBSCRIPTION_CLIENT_HPP_
