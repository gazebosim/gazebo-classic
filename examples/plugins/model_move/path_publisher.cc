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
#include <iostream>
#include <math.h>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;

int main(int argc, char * argv[])
{
  msgs::PoseAnimation msg;

  msg.set_model_name("box");
  msgs::Pose *p = msg.add_pose();
  msgs::Set(p, math::Pose(5, 5, 0, 0, 0, 0));
  p = msg.add_pose();
  msgs::Set(p, math::Pose(5, -5, 0, 0, 0, 0));
  p = msg.add_pose();
  msgs::Set(p, math::Pose(0, 0, 0, 0, 0, 0));

  transport::init();
  transport::run();
  transport::NodePtr node(new gazebo::transport::Node());
  node->Init("default");

  gazebo::transport::PublisherPtr pathPub =
    node->Advertise<msgs::PoseAnimation>("/gazebo/default/pose_animation");
  std::cout << "Waiting for connection...\n";
  pathPub->WaitForConnection();
  pathPub->Publish(msg);

  std::cout << "Path published!\n\n";

  gazebo::transport::fini();
  return 0;
}
