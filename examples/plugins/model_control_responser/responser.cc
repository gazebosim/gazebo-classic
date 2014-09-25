/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#include <iostream>

// Create our node for communication
gazebo::transport::NodePtr node;
gazebo::transport::PublisherPtr pub;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstControlRequestPtr &_msg)
{
  // Dump the message contents to stdout.
  std::cout << _msg->DebugString();

  // get request data

  // compute joint torques

  // publish joint torques
  gazebo::msgs::ControlResponse res;

  res.set_name("response");
  res.clear_torques();
  for (unsigned int i = 0; i < _msg->joint_pos().size(); ++i)
  {
    std::cout << i << "\n";
    res.add_torques(0.1);
  }
  pub->Publish(res);
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::setupClient(_argc, _argv);

  node.reset(new gazebo::transport::Node());
  node->Init();
  pub = node->Advertise<gazebo::msgs::ControlResponse>("~/ur10/control_response");

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub =
    node->Subscribe("~/ur10/control_request", cb);

  // Busy wait loop...replace with your own code as needed.
  while (true)
  {
    gazebo::common::Time::MSleep(1000);
    std::cout << ".";
  }

  // Make sure to shut everything down.
  gazebo::shutdown();
}
