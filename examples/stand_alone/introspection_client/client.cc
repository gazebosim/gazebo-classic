/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <chrono>
#include <iostream>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/util/IntrospectionClient.hh>
#include <ignition/transport.hh>

auto last = std::chrono::steady_clock::now();

/////////////////////////////////////////////////
void cb(const gazebo::msgs::Param_V &_msg)
{
  // Print the message at 1 Hz (wall-clock time).
  auto now = std::chrono::steady_clock::now();
  auto elapsed = now - last;
  if (std::chrono::duration_cast<std::chrono::milliseconds>(
        elapsed).count() >= 1000)
  {
    std::cout << _msg.DebugString() << std::endl;
    last = now;
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Use the introspection service for finding the "sim_time" item.
  gazebo::util::IntrospectionClient client;

  // Wait for the managers to come online
  std::set<std::string> managerIds = client.WaitForManagers(
      std::chrono::seconds(2));

  if (managerIds.empty())
  {
    std::cerr << "No introspection managers detected." << std::endl;
    std::cerr << "Is a gzserver running?" << std::endl;
    return -1;
  }

  // Pick up the first manager.
  std::string id = *managerIds.begin();
  std::string item = "data://world/default?p=time/sim_time";

  if (!client.IsRegistered(id, item))
  {
    std::cerr << "The sim_time item is not registered on the manager.\n";
    return -1;
  }

  // Create a filter for watching the "sim_time" item.
  std::string filterId, topic;
  if (!client.NewFilter(id, {item}, filterId, topic))
  {
    return -1;
  }

  // Let's subscribe to the topic for receiving updates.
  ignition::transport::Node node;
  node.Subscribe(topic, cb);

  /// zZZZ.
  ignition::transport::waitForShutdown();

  return 0;
}
