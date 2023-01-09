// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// std
#include <memory>

// local
#include "romea_cmd_mux/cmd_mux.hpp"
#include "romea_cmd_mux_utils/cmd_mux_interface.hpp"

class TestableCmdMux : public romea::CmdMux
{
public:
  explicit TestableCmdMux(const rclcpp::NodeOptions & options)
  : CmdMux(options)
  {
  }

  std::shared_ptr<rclcpp::Node> get_node()
  {
    return node_;
  }
};

class TestCmdMuxServices : public ::testing::Test
{
public:
  TestCmdMuxServices()
  : cmd_mux()
  {
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    rclcpp::NodeOptions cmd_mux_no;
    cmd_mux_no.arguments({"--ros-args", "-p", "topics_type:=std_msgs/String"});
    cmd_mux = std::make_unique<TestableCmdMux>(cmd_mux_no);

    cmd_mux_subscription_client = std::make_unique<romea::CmdMuxSubscriptionClient>(
      cmd_mux->get_node());
    cmd_mux_unsubscription_client = std::make_unique<romea::CmdMuxUnsubscriptionClient>(
      cmd_mux->get_node());
  }


  std::unique_ptr<TestableCmdMux> cmd_mux;
  std::unique_ptr<romea::CmdMuxSubscriptionClient> cmd_mux_subscription_client;
  std::unique_ptr<romea::CmdMuxUnsubscriptionClient> cmd_mux_unsubscription_client;
};

TEST_F(TestCmdMuxServices, testSubscription)
{
  EXPECT_NO_THROW(cmd_mux_subscription_client->subscribe("/foo", 100, 0.5));
  EXPECT_NO_THROW(cmd_mux_subscription_client->subscribe("/bar", 110, 0.5));
  auto topics = cmd_mux->get_node()->get_topic_names_and_types();
  EXPECT_TRUE(topics.find("/foo") != topics.end());
  EXPECT_TRUE(topics.find("/bar") != topics.end());
}

TEST_F(TestCmdMuxServices, testSubscriptionFailedBecauseTopicIsAlreadySubscribed)
{
  EXPECT_NO_THROW(cmd_mux_subscription_client->subscribe("/foo", 100, 0.5));
  EXPECT_THROW(cmd_mux_subscription_client->subscribe("/foo", 110, 0.5), std::runtime_error);

  auto topics = cmd_mux->get_node()->get_topic_names_and_types();
  EXPECT_TRUE(topics.find("/foo") != topics.end());
}

TEST_F(TestCmdMuxServices, testSubscriptionFailedBecausePriorityIsAlreadyUsed)
{
  EXPECT_NO_THROW(cmd_mux_subscription_client->subscribe("/foo", 100, 0.5));
  EXPECT_THROW(cmd_mux_subscription_client->subscribe("/bar", 100, 0.5), std::runtime_error);

  auto topics = cmd_mux->get_node()->get_topic_names_and_types();
  EXPECT_TRUE(topics.find("/foo") != topics.end());
  EXPECT_TRUE(topics.find("/bar") == topics.end());
}

TEST_F(TestCmdMuxServices, testUnsubscription)
{
  EXPECT_NO_THROW(cmd_mux_subscription_client->subscribe("/foo", 100, 0.5));
  EXPECT_NO_THROW(cmd_mux_unsubscription_client->unsubscribe("/foo"));
  auto topics = cmd_mux->get_node()->get_topic_names_and_types();
  EXPECT_TRUE(topics.find("/foo") == topics.end());
}

TEST_F(TestCmdMuxServices, testUnsubscriptionWhenNoSubscriptionIsDone)
{
  EXPECT_THROW(cmd_mux_unsubscription_client->unsubscribe("/foo"), std::runtime_error);
  EXPECT_NO_THROW(cmd_mux_subscription_client->subscribe("/foo", 100, 0.5));
  EXPECT_THROW(cmd_mux_unsubscription_client->unsubscribe("/bar"), std::runtime_error);
  auto topics = cmd_mux->get_node()->get_topic_names_and_types();
  EXPECT_TRUE(topics.find("/foo") != topics.end());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
