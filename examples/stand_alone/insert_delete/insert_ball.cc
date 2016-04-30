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

#include <iostream>

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Start the client
  gazebo::client::setup(_argc, _argv);

  // SDF string for a bouncy ball
  std::ostringstream ballStr;
  ballStr <<
      "<sdf version='" << SDF_VERSION << "'>" <<
      "<model name='ball'>"
      "  <link name='link'>"
      "    <inertial>"
      "      <mass>0.213</mass>"
      "      <inertia>"
      "        <ixx>0.003226667</ixx>"
      "        <ixy>0</ixy>"
      "        <ixz>0</ixz>"
      "        <iyy>0.003226667</iyy>"
      "        <iyz>0</iyz>"
      "        <izz>0.003226667</izz>"
      "      </inertia>"
      "    </inertial>"
      "    <visual name='visual'>"
      "      <geometry>"
      "        <sphere>"
      "          <radius>0.127</radius>"
      "        </sphere>"
      "      </geometry>"
      "    </visual>"
      "    <collision name='collision'>"
      "      <geometry>"
      "        <sphere>"
      "          <radius>0.127</radius>"
      "        </sphere>"
      "      </geometry>"
      "      <surface>"
      "        <bounce>"
      "          <restitution_coefficient>0.15937</restitution_coefficient>"
      "          <threshold>0</threshold>"
      "        </bounce>"
      "        <contact>"
      "          <ode>"
      "            <max_vel>100</max_vel>"
      "            <min_depth>0</min_depth>"
      "          </ode>"
      "        </contact>"
      "      </surface>"
      "    </collision>"
      "  </link>"
      "</model>"
      "</sdf>";

  // Spawning pose
  ignition::math::Pose3d pose(0, 0, 5, 0, 0, 0);

  // Request insertion
  gazebo::transport::RequestEntityInsert(ballStr.str(), pose);

  // Close the client"
  gazebo::client::shutdown();
}
