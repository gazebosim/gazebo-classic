/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <gazebo/math/gzmath.hh>

#include <iostream>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to a Gazebo topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Marker>("~/marker");

  // Wait for a subscriber to connect
  pub->WaitForConnection();

  gazebo::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  markerMsg.set_id(0);
  markerMsg.set_action(gazebo::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gazebo::msgs::Marker::SPHERE);

  gazebo::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/BlueLaser");

  std::cout << "Spawning a sphere at the origin\n";
  gazebo::common::Time::Sleep(4);
  pub->Publish(markerMsg);

  std::cout << "Moving the sphere to x=0, y=0, z=1\n";
  gazebo::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 0, 1, 0, 0, 0));
  gazebo::common::Time::Sleep(4);
  pub->Publish(markerMsg);

  std::cout << "Shrinking the sphere\n";
  gazebo::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(0.2, 0.2, 0.2));
  gazebo::common::Time::Sleep(4);
  pub->Publish(markerMsg);

  std::cout << "Changing the sphere to red\n";
  matMsg->mutable_script()->set_name("Gazebo/Red");
  gazebo::common::Time::Sleep(4);
  pub->Publish(markerMsg);

  std::cout << "Adding a green cube\n";
  markerMsg.set_id(1);
  markerMsg.set_action(gazebo::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gazebo::msgs::Marker::CUBE);
  matMsg->mutable_script()->set_name("Gazebo/Green");
  gazebo::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(0.4, 0.4, 0.4));
  gazebo::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(2, 0, .5, 0, 0, 0));
  gazebo::common::Time::Sleep(4);
  pub->Publish(markerMsg);

  gazebo::common::Time::Sleep(2);

  //markerMsg.set_type(gazebo::msgs::Marker::LINE_STRIP);

  //gazebo::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(0, 0, 0.2));
  //gazebo::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(0, 1, 0.2));
  //gazebo::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(1, 0, 0.2));

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
