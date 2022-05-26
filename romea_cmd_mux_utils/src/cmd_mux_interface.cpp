#include "romea_cmd_mux_utils/cmd_mux_interface.hpp"

namespace romea
{


//-----------------------------------------------------------------------------
CmdMuxInterface::CmdMuxInterface(std::shared_ptr<rclcpp::Node> node):
  subscription_(node),
  unsubscription_(node),
  subscribed_topics_()
{

}

//-----------------------------------------------------------------------------
CmdMuxInterface::~CmdMuxInterface()
{
  for(const std::string & topic : subscribed_topics_)
  {
    unsubscription_.unsubscribe(topic);
  }
}

//-----------------------------------------------------------------------------
void CmdMuxInterface::subscribe(const std::string & topic,
                                const int & priority,
                                const double & timeout)
{
  subscription_.subscribe(topic,priority,timeout);
  subscribed_topics_.push_back(topic);
}

//-----------------------------------------------------------------------------
void CmdMuxInterface::unsubscribe(const std::string & topic)
{
  unsubscription_.unsubscribe(topic);
  subscribed_topics_.remove(topic);
}

}
