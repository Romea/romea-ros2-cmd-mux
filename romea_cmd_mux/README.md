# 1 Overview #

This package provides a generic command mutiplexer based on ideas found in [topic_tools mux](https://github.com/ros-tooling/topic_tools), [twist_mux](https://github.com/ros-teleop/twist_mux) or [cmd_vel_mux](https://github.com/kobuki-base/cmd_vel_mux) nodes.  It takes N input topics and outputs the messages from a single one according their priority and their acitivity. Finally, thanks to rclcpp's GenericSubscriber and GenericPublisher any kind of messages can be mutiplexed.    

# 2 Node #

### 2.1 Subscribed Topics ###

-  cmd_input_N (any::any)

    The Nth input topic that mux is subscribed to (specified via connect service)

### 2.2 Published Topics ###

- cmd_out (any::any)

   Messages from the selected input are published into this topic

### 2.3 Service servers ###

- subscribe (romea_cmd_mux_msgs::srv::Subscribe)

  This service is used to try to add an input topic according three informations :  

     -  topic : name of the topic
     -  timeout : messages timeout in seconds. If no message arrives after the timeout, another active topic is selected.
     -  priority : priority of the topic from 0 to 255. The higher the more priority it has over the others.

  Subscription request can failed if an input topic is already resgister with the same name or if a input topic has already the same priority.      

- unsubscribe (romea_cmd_mux_msgs::srv::Unsubscribe)

  This service is used to remove subscription at an input topic.

### 2.3 Parameters ###

- topics_type (string, default None)

  type of the messages to be multiplexed (format pkg/msg)