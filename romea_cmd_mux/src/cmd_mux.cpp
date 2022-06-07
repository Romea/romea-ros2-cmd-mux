#include "romea_cmd_mux/cmd_mux.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
CmdMux::CmdMux(const rclcpp::NodeOptions & options):
  mutex_(),
  node_(std::make_shared<rclcpp::Node>("cmd_mux",options)),
  publisher_(),
  subscribers_(),
  subscribe_service_(),
  unsubscribe_service_(),
  topics_type_()
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  node_->declare_parameter<std::string>("topics_type");

  if(!node_->get_parameter("topics_type",topics_type_))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),"Parameter topics_type is not defined, cannot initialize cmd_mux");
    return;
  }

  subscribe_service_=node_->create_service<romea_cmd_mux_msgs::srv::Subscribe>(
        "~/subscribe", std::bind(&CmdMux::subscribe_callback_,this,_1,_2));

  unsubscribe_service_=node_->create_service<romea_cmd_mux_msgs::srv::Unsubscribe>(
        "~/unsubscribe", std::bind(&CmdMux::unsubscribe_callback_,this,_1,_2));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  publisher_ = node_->create_generic_publisher("~/out",topics_type_,qos);

}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
CmdMux::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
//void CmdMux::diagnosticCallback_(ros::TimerEvent & event)
//{
//}

//-----------------------------------------------------------------------------
void CmdMux::subscribe_callback_(const SubscribeRequestSharedPtr  request,
                                 SubscribeResponseSharedPtr response)
{
  using std::placeholders::_1;
  std::lock_guard<std::mutex> lock(mutex_);
  using Response = romea_cmd_mux_msgs::srv::Subscribe::Response;

  auto lambda = [&request](const std::pair<unsigned char, Subscriber> & s)
  {
    return s.second.sub->get_topic_name()==request->topic;
  };


  auto itTopic = std::find_if(std::cbegin(subscribers_),
                              std::cend(subscribers_),
                              lambda);

  if(itTopic!=std::cend(subscribers_))
  {
    response->result = Response::REJECTED_TOPIC_ALREADY_SUBSCRIBED;
    return;
  }


  auto itPriority = subscribers_.find(request->priority);

  if(itPriority != std::cend(subscribers_))
  {
    response->result = Response::REJECTED_PRIORITY_ALREADY_USED;
    return;
  }

  auto & subscriber = subscribers_[request->priority];

  subscriber.timeout.from_seconds(request->timeout);
  auto f =  std::bind(&CmdMux::publish_callback_,this,_1,request->priority);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  subscriber.sub = node_->create_generic_subscription(request->topic,topics_type_,qos,f);

  response->result = Response::ACCEPTED;
}

//-----------------------------------------------------------------------------
void CmdMux::unsubscribe_callback_(const UnsubscribeRequestSharedPtr request,
                                   UnsubscribeResponseSharedPtr response)
{
  std::lock_guard<std::mutex> lock(mutex_);


  auto lambda = [&](const std::pair<unsigned char, Subscriber> & s)
  {
    return s.second.sub->get_topic_name()==request->topic;
  };

  auto it = std::find_if(std::cbegin(subscribers_),
                         std::cend(subscribers_),
                         lambda);

  if(it!=subscribers_.end())
  {
    subscribers_.erase(it);
    response->result=romea_cmd_mux_msgs::srv::Unsubscribe::Response::ACCEPTED;
  }
  else
  {
    response->result = romea_cmd_mux_msgs::srv::Unsubscribe::Response::REJECTED;
  }

}



//-----------------------------------------------------------------------------
void CmdMux::publish_callback_(MsgSharedPtr msg,unsigned char priotity)
{
  std::lock_guard<std::mutex> lock(mutex_);

  rclcpp::Time now = node_->get_clock()->now();
  auto it = subscribers_.find(priotity);
  (*it).second.msg_stamp = now;
  if(has_highest_priority_(it,now))
  {
    publisher_->publish(*msg);
  }
}

//-----------------------------------------------------------------------------
bool CmdMux::has_highest_priority_(SubscriberMap::iterator it ,const rclcpp::Time & now)
{
  while(++it != subscribers_.end())
  {
    if(now-(*it).second.msg_stamp<(*it).second.timeout)
    {
      return false;
    }
  }
  return true;
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(romea::CmdMux)
