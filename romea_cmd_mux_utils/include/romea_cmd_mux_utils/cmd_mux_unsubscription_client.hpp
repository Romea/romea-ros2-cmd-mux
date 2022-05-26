#ifndef CmdMuxClient_HPP
#define CmdMuxClient_HPP

//ros
#include <rclcpp/rclcpp.hpp>
#include <romea_cmd_mux_msgs/srv/unsubscribe.hpp>

//local
#include "visibility_control.h"

namespace romea
{

class CmdMuxUnsubscriptionClient
{

private:

  enum Result {
    ACCEPTED,
    REJECTED,
    FAIL_TO_CALL_SERVICE,
    FAIL_TO_SEND_REQUEST
  };

  using Service = romea_cmd_mux_msgs::srv::Unsubscribe;
public:

  ROMEA_CMD_MUX_UTILS_PUBLIC
  CmdMuxUnsubscriptionClient(std::shared_ptr<rclcpp::Node> node);

  ROMEA_CMD_MUX_UTILS_PUBLIC
  void unsubscribe(const std::string & topic);

private :

  Result unsubscribe_(const std::string & topic);

private:

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<Service>::SharedPtr client_;
};


}

#endif
