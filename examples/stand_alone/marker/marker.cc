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
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <gazebo/common/Time.hh>

#include <iostream>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  ignition::transport::Node node;

  // Create the marker message
  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  // 0 has a special meaning of representing an id which hasn't been set
  // Thus, when 0 is given as an argument to set_id, it generates a random (but unique) id,
  // and sets the marker's id value to that randomly generated id.
  // A new marker is created at every invocation of set_id(0).
  markerMsg.set_id(1);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::SPHERE);

  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/BlueLaser");

  // The rest of this function adds different shapes and/or modifies shapes.
  // Read the terminal statements to figure out what each node.Request
  // call accomplishes.

  std::cout << "Spawning a sphere at the origin\n";
  gazebo::common::Time::Sleep(4);
  node.Request("/marker", markerMsg);

  std::cout << "Moving the sphere to x=0, y=0, z=1\n";
  gazebo::common::Time::Sleep(4);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(0, 0, 1, 0, 0, 0));
  node.Request("/marker", markerMsg);

  std::cout << "Shrinking the sphere\n";
  gazebo::common::Time::Sleep(4);
  ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(0.2, 0.2, 0.2));
  node.Request("/marker", markerMsg);

  std::cout << "Changing the sphere to red\n";
  gazebo::common::Time::Sleep(4);
  matMsg->mutable_script()->set_name("Gazebo/Red");
  node.Request("/marker", markerMsg);

  std::cout << "Adding a green box\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(2);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::BOX);
  matMsg->mutable_script()->set_name("Gazebo/Green");
  ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(1.0, 1.0, 1.0));
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(2, 0, .5, 0, 0, 0));
  node.Request("/marker", markerMsg);

  std::cout << "Change the green box to a cylinder\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_type(ignition::msgs::Marker::CYLINDER);
  node.Request("/marker", markerMsg);

  std::cout << "Adding a line between the sphere and cylinder\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(3);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 0, 1.1));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 0, 0.5));
  node.Request("/marker", markerMsg);

  std::cout << "Adding a square around the origin\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(4);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
  ignition::msgs::Set(markerMsg.mutable_point(0),
      ignition::math::Vector3d(0.5, 0.5, 0.05));
  ignition::msgs::Set(markerMsg.mutable_point(1),
      ignition::math::Vector3d(0.5, -0.5, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(-0.5, -0.5, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(-0.5, 0.5, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0.5, 0.5, 0.05));
  node.Request("/marker", markerMsg);

  std::cout << "Adding 100 points inside the square\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(5);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::POINTS);
  markerMsg.clear_point();
  for (int i = 0; i < 100; ++i)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(
          ignition::math::Rand::DblUniform(-0.49, 0.49),
          ignition::math::Rand::DblUniform(-0.49, 0.49),
          0.05));
  }
  node.Request("/marker", markerMsg);

  std::cout << "Adding HELLO at 0, 0, 2\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(6);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TEXT);
  markerMsg.set_text("HELLO");
  ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(0.2, 0.2, 0.2));
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 0, 2, 0, 0, 0));
  node.Request("/marker", markerMsg);

  std::cout << "Adding a semi-circular triangle fan\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(7);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 1.5, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.05));
  double radius = 2;
  for (double t = 0; t <= M_PI; t+= 0.01)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(radius * cos(t), radius * sin(t), 0.05));
  }
  node.Request("/marker", markerMsg);

  std::cout << "Adding two triangles using a triangle list\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(8);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, -1.5, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.05));

  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 1, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 2, 0.05));

  node.Request("/marker", markerMsg);

  std::cout << "Adding a rectangular triangle strip\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_id(9);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(-2, -2, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 0, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 1, 0.05));

  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 2, 0.05));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 2, 0.05));

  node.Request("/marker", markerMsg);

  std::cout << "Delete all the markers\n";
  gazebo::common::Time::Sleep(4);
  markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
  node.Request("/marker", markerMsg);
}
