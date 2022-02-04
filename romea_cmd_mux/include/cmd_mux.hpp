#ifndef CmdMux_HPP
#define CmdMux_HPP


//ros
#include <romea_cmd_mux_msgs/srv/connect.hpp>
#include <romea_cmd_mux_msgs/srv/disconnect.hpp>

//local
#include "subscriber_diagnostic.hpp"
#include "subscriber.hpp"

//std
#include <map>
#include <mutex>


namespace romea
{

class CmdMux : public rclcpp::Node
{

protected :

  //using SubscriberCallbackFunction = boost::function<void(const topic_tools::ShapeShifter::ConstPtr & msg)>;
  using SubscriberMap = std::map<unsigned char, Subscriber> ;
  using ConnectServiceSharedPtr = rclcpp::Service<romea_cmd_mux_msgs::srv::Connect>::SharedPtr;
  using ConnectRequestSharedPtr = std::shared_ptr<romea_cmd_mux_msgs::srv::Connect::Request>;
  using ConnectResponseSharedPtr = std::shared_ptr<romea_cmd_mux_msgs::srv::Connect::Response>;
  using DisconnectServiceSharedPtr = rclcpp::Service<romea_cmd_mux_msgs::srv::Disconnect>::SharedPtr;
  using DisconnectRequestSharedPtr = std::shared_ptr<romea_cmd_mux_msgs::srv::Disconnect::Request>;
  using DisconnectResponseSharedPtr = std::shared_ptr<romea_cmd_mux_msgs::srv::Disconnect::Response>;
  using PublisherSharedPtr =  rclcpp::GenericPublisher::SharedPtr;
  using MsgSharedPtr = std::shared_ptr<rclcpp::SerializedMessage>;

public:

  CmdMux();
  virtual ~CmdMux()=default;

  virtual void onInit();

//  void diagnosticCallback_(ros::TimerEvent & event);


protected:


  bool hasHighestPriority_(SubscriberMap::iterator it ,const rclcpp::Time & now);

  void connectCallback_(const ConnectRequestSharedPtr  request,
                        ConnectResponseSharedPtr response);

  void disconnectCallback_(const DisconnectRequestSharedPtr request,
                           DisconnectResponseSharedPtr response);

  void publishCallback_(MsgSharedPtr msg, unsigned char priotity);

protected :

  std::mutex mutex_;
  PublisherSharedPtr publisher_;
  SubscriberMap subscribers_;


  ConnectServiceSharedPtr connect_service_;
  DisconnectServiceSharedPtr disconnect_service_;
  std::string topics_type_;
  rclcpp::QoS qos_;
//  std::string publisher_topic_name_;
//  bool is_publisher_initialized_;

};

}

#endif
