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

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <iostream>

gazebo::transport::PublisherPtr controlPub;
double state;

void StateIn(ConstAnyPtr &_msg)
{
  gzerr << "got state.\n";
  state = _msg->double_value();

  // do some calculations


  // publish new command
  gazebo::msgs::Any msg;
  msg.set_type(gazebo::msgs::Any_ValueType_DOUBLE);
  msg.set_double_value(10.);
  controlPub->Publish(msg);
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  if (_argc <= 2)
  {
    std::cerr << "Invalid usage.\n\n"
      << "Usage:\n    robot_controller <Hz> <model name>\n\n"
      << "Example:\n    robot_controller 100 ur10_1\n";
    return -1;
  }

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  std::string modelName = _argv[2];
  std::string prefix = "~/" + modelName + "/";

  // publish control
  controlPub =
    node->Advertise<gazebo::msgs::Any>(prefix + "control");

  // listen to incoming states
  gazebo::transport::SubscriberPtr stateSub =
    node->Subscribe(prefix + "state", &StateIn);

  // Wait for a subscriber to connect
  controlPub->WaitForConnection();

  while (true)
  {
    usleep(1000000);
  };

  // Make sure to shut everything down.
  gazebo::client::shutdown();

  return 0;
}
