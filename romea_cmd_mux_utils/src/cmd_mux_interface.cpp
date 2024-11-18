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


// std
#include <memory>
#include <string>

// local
#include "romea_cmd_mux_utils/cmd_mux_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void CmdMuxInterface::subscribe(
  const std::string & topic,
  const int & priority,
  const double & timeout)
{
  subscription_.subscribe(topic, priority, timeout);
  subscribed_topics_.push_back(topic);
}

//-----------------------------------------------------------------------------
void CmdMuxInterface::unsubscribe(const std::string & topic)
{
  if (std::find(subscribed_topics_.begin(), subscribed_topics_.end(), topic) !=
    subscribed_topics_.end())
  {
    unsubscription_.unsubscribe(topic);
    subscribed_topics_.remove(topic);
  }
}

}  // namespace romea
