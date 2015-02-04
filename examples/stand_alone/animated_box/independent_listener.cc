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

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#include <iostream>

/////////////////////////////////////////////////
// Function is called every time a message is received.
void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
  std::cout << posesStamped->DebugString();

  ::google::protobuf::int32 sec = posesStamped->time().sec();
  ::google::protobuf::int32 nsec = posesStamped->time().nsec();
  std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;

  for (int i =0; i < posesStamped->pose_size(); ++i)
  {
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
    std::string name = pose.name();
    if (name == std::string("box"))
    {
      const ::gazebo::msgs::Vector3d &position = pose.position();

      double x = position.x();
      double y = position.y();
      double z = position.z();

      std::cout << "Read position: x: " << x
          << " y: " << y << " z: " << z << std::endl;
    }
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::setupClient(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo pose info topic
  gazebo::transport::SubscriberPtr sub =
  node->Subscribe("~/pose/info", posesStampedCallback);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::shutdown();
}
