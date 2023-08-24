// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ROMEA_CMD_MUX_UTILS__CMD_MUX_INTERFACE_HPP_
#define ROMEA_CMD_MUX_UTILS__CMD_MUX_INTERFACE_HPP_

// std
#include <memory>
#include <list>
#include <string>

// romea
#include "romea_cmd_mux_utils/cmd_mux_subscription_client.hpp"
#include "romea_cmd_mux_utils/cmd_mux_unsubscription_client.hpp"

namespace romea
{

class CmdMuxInterface
{
public:
  template<typename Node>
  ROMEA_CMD_MUX_UTILS_PUBLIC
  explicit CmdMuxInterface(std::shared_ptr<Node> node);

  ROMEA_CMD_MUX_UTILS_PUBLIC
  ~CmdMuxInterface() = default;

  ROMEA_CMD_MUX_UTILS_PUBLIC
  void subscribe(
    const std::string & topic,
    const int & priority,
    const double & timeout);

  ROMEA_CMD_MUX_UTILS_PUBLIC
  void unsubscribe(const std::string & topic);

private:
  CmdMuxSubscriptionClient subscription_;
  CmdMuxUnsubscriptionClient unsubscription_;
  std::list<std::string> subscribed_topics_;
};


//-----------------------------------------------------------------------------
template<typename Node>
CmdMuxInterface::CmdMuxInterface(std::shared_ptr<Node> node)
: subscription_(node),
  unsubscription_(node),
  subscribed_topics_()
{
}

}  // namespace romea

#endif  // ROMEA_CMD_MUX_UTILS__CMD_MUX_INTERFACE_HPP_
