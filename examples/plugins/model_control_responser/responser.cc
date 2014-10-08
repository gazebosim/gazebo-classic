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

#include <ignition/transport.hh>
#include <gazebo/msgs/msgs.hh>

// include dartcore stuff here

#include <iostream>

// Create our node for communication
boost::shared_ptr<ignition::transport::Node> node;

// task space classes
using namespace std;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(const std::string &_topic, const gazebo::msgs::ControlRequest &_req,
  gazebo::msgs::ControlResponse &_res, bool &_result)
{
  // Dump the message contents to stdout.
  std::cout << _req.DebugString();

  // get request data from _req.joint_pos, _req.joint_vel
  /* simbody example
  for (int i=0; i < UR10::NumCoords; ++i) {
      // std::cout << "num coords: " << UR10::NumCoords << "\n";
      const UR10::Coords coord = UR10::Coords(i);
      // skip the world_joint (i+1)
      m_modelRobot.setJointAngle(m_modelState, coord, _req.joint_pos(i+1));
      m_modelRobot.setJointRate(m_modelState, coord, _req.joint_vel(i+1));
  }
  */

  // heavy lifting: compute torques based on joint and robot states

  // publish joint torques
  _res.set_name("response");
  _res.clear_torques();
  /* example from simbody
  _res.add_torques(0); // for the world_joint
  for (unsigned int i = 0; i < UR10::NumCoords; ++i)
  {
    // std::cout << i << " : " << tau[i] << "\n";
    _res.add_torques(tau[i]);
  }
  */
  _result = true;
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  node.reset(new ignition::transport::Node());

  // initialize model state (internal copy to controller)

  node->Advertise("/ur10/control_request", cb);

  int c = 0;
  // Busy wait loop...replace with your own code as needed.
  while(c != 'q')
  {
    std::cout << "enter q to quit, g to toggle gravity: ";
    c = getchar();
    if (c == 'g')
      m_modelTasks.toggleGravityComp();
  }
  std::cout << "exit!";
}
