/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <gazebo/common/Color.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  if (_argc <= 2)
  {
    std::cerr << "Please provide the stop_light_post model name and the desired"
      << " light color. Possible lights are: red / yellow / green. Give an"
      << " invalid color to shut down all lights.\n\n"
      << "Usage:\n    stop_light <model name> <color>\n\n"
      << "Example:\n    stop_light stop_light_1 green\n";
    return -1;
  }

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to a Gazebo topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Visual>("/gazebo/default/visual");

  // Wait for a subscriber to connect
  pub->WaitForConnection();

  // Colors
  auto red = std::make_pair(gazebo::common::Color::Red, "red");
  auto yellow = std::make_pair(gazebo::common::Color::Yellow, "yellow");
  auto green = std::make_pair(gazebo::common::Color::Green, "green");
  auto black = gazebo::common::Color::Black;

  std::string color = _argv[2];

  // Publish for each light
  std::string modelName = _argv[1];
  for (auto c : {red, yellow, green})
  {
    for (auto l : {"right_light", "center_light"})
    {
      gazebo::msgs::Visual msg;
      msg.set_type(gazebo::msgs::Visual::VISUAL);
      msg.set_parent_name(modelName + "::" + l + "::link");

      msg.set_name(modelName + "::" + l + "::link::" + c.second);

      auto matMsg = msg.mutable_material();
      if (c.second == color)
        gazebo::msgs::Set(matMsg->mutable_emissive(), c.first);
      else
        gazebo::msgs::Set(matMsg->mutable_emissive(), black);

      pub->Publish(msg);
    }
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
