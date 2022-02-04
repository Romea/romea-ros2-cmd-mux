#include "cmd_mux.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
CmdMux::CmdMux():
  Node("cmd_mux"),
  mutex_(),
  publisher_(),
  subscribers_(),
  connect_service_(),
  topics_type_(),
  qos_(10)
//  is_publisher_initialized_(false)
{
  declare_parameter<std::string>("topics_type");
}

//-----------------------------------------------------------------------------
void CmdMux::onInit()
{

  using std::placeholders::_1;
  using std::placeholders::_2;


  if(!get_parameter("topics_type",topics_type_))
  {
    RCLCPP_ERROR_STREAM(get_logger(),"Parameter topics_type is not defined, cannot initialize cmd_mux");
    return;
  }

  connect_service_=create_service<romea_cmd_mux_msgs::srv::Connect>(
        "connect", std::bind(&CmdMux::connectCallback_,this,_1,_2));

  disconnect_service_=create_service<romea_cmd_mux_msgs::srv::Disconnect>(
        "disconnect", std::bind(&CmdMux::disconnectCallback_,this,_1,_2));

  publisher_ = create_generic_publisher("out",topics_type_,qos_);
}


//-----------------------------------------------------------------------------
//void CmdMux::diagnosticCallback_(ros::TimerEvent & event)
//{
//}

//-----------------------------------------------------------------------------
void CmdMux::connectCallback_(const ConnectRequestSharedPtr  request,
                              ConnectResponseSharedPtr response)
{
  using std::placeholders::_1;
  std::lock_guard<std::mutex> lock(mutex_);

  auto lambda = [&request](const std::pair<unsigned char, Subscriber> & s)
  {
    return s.second.sub->get_topic_name()==request->topic;
  };


  auto itTopic = std::find_if(std::cbegin(subscribers_),
                              std::cend(subscribers_),
                              lambda);

  auto itPriority = subscribers_.find(request->priority);

  if(itTopic==std::cend(subscribers_) && itPriority == std::cend(subscribers_))
  {
    auto & subscriber = subscribers_[request->priority];

    subscriber.timeout.from_seconds(request->timeout);
    auto f =  std::bind(&CmdMux::publishCallback_,this,_1,request->priority);
    subscriber.sub = create_generic_subscription(request->topic,topics_type_,qos_,f);

    response->success = true;
  }
  else
  {
    response->success = (itTopic==itPriority);
  }
}

//-----------------------------------------------------------------------------
void CmdMux::disconnectCallback_(const DisconnectRequestSharedPtr request,
                                 DisconnectResponseSharedPtr response)
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
    response->success = true;
  }
  else
  {
    response->success = false;
  }

}



//-----------------------------------------------------------------------------
void CmdMux::publishCallback_(MsgSharedPtr msg,unsigned char priotity)
{
  std::lock_guard<std::mutex> lock(mutex_);

//  if(!is_publisher_initialized_)
//  {
//    publisher_= msg->advertise(getNodeHandle(),"cmd_out",1);
//    is_publisher_initialized_=true;
//  }

  rclcpp::Time now = get_clock()->now();
  auto it = subscribers_.find(priotity);
  (*it).second.msg_stamp = now;
  if(hasHighestPriority_(it,now))
  {
    publisher_->publish(*msg);
  }
}

//-----------------------------------------------------------------------------
bool CmdMux::hasHighestPriority_(SubscriberMap::iterator it ,const rclcpp::Time & now)
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

