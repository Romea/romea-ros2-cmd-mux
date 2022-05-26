#ifndef CmdMuxInterface_HPP
#define CmdMuxInterface_HPP

#include "cmd_mux_subscription_client.hpp"
#include "cmd_mux_unsubscription_client.hpp"

namespace romea
{

class CmdMuxInterface
{

public:

  ROMEA_CMD_MUX_UTILS_PUBLIC
  CmdMuxInterface(std::shared_ptr<rclcpp::Node> node);

  ROMEA_CMD_MUX_UTILS_PUBLIC
  ~CmdMuxInterface();

  ROMEA_CMD_MUX_UTILS_PUBLIC
  void subscribe(const std::string & topic,
                 const int & priority,
                 const double & timeout);

  ROMEA_CMD_MUX_UTILS_PUBLIC
  void unsubscribe(const std::string & topic);

private:

  CmdMuxSubscriptionClient subscription_;
  CmdMuxUnsubscriptionClient unsubscription_;
  std::list<std::string> subscribed_topics_;

};


}

#endif
