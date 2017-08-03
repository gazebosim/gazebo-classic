/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <iostream>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  if (_argc != 7)
  {
    std::cerr << "Invalid usage.\n\n"
      << "Usage:\n    harness_attach "
      << "<float_x> <float_y> <float_z> "
      << "<float_roll> <float_pitch> <float_yaw>\n\n"
      << "Example:\n    harness_attach 0 0 1.5 0 0 1.5708\n";
    return -1;
  }

  // create pose object
  ignition::math::Pose3d pose(
    std::stof(_argv[1]),
    std::stof(_argv[2]),
    std::stof(_argv[3]),
    std::stof(_argv[4]),
    std::stof(_argv[5]),
    std::stof(_argv[6]));

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to a Gazebo topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Pose>("~/box/harness/attach");

  // Wait for a subscriber to connect
  pub->WaitForConnection();

  // Convert to a pose message
  gazebo::msgs::Pose msg;
  gazebo::msgs::Set(&msg, pose);

  pub->Publish(msg);

  // Make sure to shut everything down.
  gazebo::client::shutdown();

  return 0;
}
