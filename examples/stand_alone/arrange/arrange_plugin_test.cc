/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <gazebo/gazebo_client.hh>
#include <vector>
#include <iostream>

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

int main(int _argc, char** _argv)
{
  gazebo::client::setup(_argc, _argv);

  std::vector<std::string> arrangementNames;

  arrangementNames.push_back("default");
  arrangementNames.push_back("cricket_ball");
  arrangementNames.push_back("wood_blocks");
  arrangementNames.push_back("wood_blocks_stack");

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to a Gazebo topic
  gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::GzString>
                                                      ("~/arrange");

  int i = 0;
  while (true)
  {
    std::cout << "Setting arrangement to: " << arrangementNames[i] << std::endl;
    gazebo::msgs::GzString msg;
    msg.set_data(arrangementNames[i]);
    pub->Publish(msg);

    i = (i+1) % arrangementNames.size();
    gazebo::common::Time::MSleep(1000);
  }

  gazebo::client::shutdown();
}
