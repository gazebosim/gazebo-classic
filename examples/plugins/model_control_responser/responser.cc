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

#include <Simbody.h>

#include "Atlas.h"
#include "task_measure.h"

#include <iostream>

// Create our node for communication
boost::shared_ptr<ignition::transport::Node> node;

// task space classes
using namespace SimTK;
using namespace std;

// global for now
Atlas                m_modelRobot("atlas_v4_upper.urdf");
TasksMeasure<Vector> m_modelTasks(m_modelRobot);
State                m_modelState;
// Visualizer           *m_viz;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void callback(const std::string &_topic,
  const gazebo::msgs::ControlRequest &_req,
  gazebo::msgs::ControlResponse &_res, bool &_result)
{
  // Dump the message contents to stdout.
  // std::cout << "responding\n";
  // std::cout << _req.DebugString();

  std::vector<unsigned int> jIndex;
  std::vector<unsigned int> uIndex, qIndex;
  jIndex.resize(_req.joint_names().size());
  uIndex.resize(_req.joint_names().size());
  qIndex.resize(_req.joint_names().size());
  // build map
  for (unsigned int i=0; i < _req.joint_names().size(); ++i) {
    // find mobilized body id that matches joint name
    // find mapping between mobilized body u indices and i indices
    const URDFRobot &robot = m_modelRobot.getURDFRobot();
    const URDFJointInfo &jInfo = robot.joints.getJoint(_req.joint_names(i));
    const MobilizedBody &mobod = jInfo.mobod;
    unsigned int nU = mobod.getNumU(m_modelState);
    unsigned int nQ = mobod.getNumQ(m_modelState);
    unsigned int u0 = mobod.getFirstUIndex(m_modelState);
    unsigned int q0 = mobod.getFirstQIndex(m_modelState);
    assert(nU == 1);
    assert(nQ == 1);
    // this example works for single DOF joints
    uIndex[i] = u0;
    qIndex[i] = q0;
    jIndex[u0] = i;
  }

  // get request data
  for (unsigned int i=0; i < uIndex.size(); ++i) {
      m_modelRobot.setJointAngle(m_modelState, QIndex(qIndex[i]),
                                 _req.joint_pos(i));
      m_modelRobot.setJointRate(m_modelState, UIndex(uIndex[i]),
                                 _req.joint_vel(i));
  }

  // get target
  Vec3 target_pos(_req.target_pos().x(),
                  _req.target_pos().y(),
                  _req.target_pos().z());
  m_modelTasks.setTarget(target_pos);

  // get endeffector position from message
  gazebo::msgs::Vector3d endEffectorPos = _req.end_effector_pos();

  SimTK::Vec3 e(endEffectorPos.x(),
                endEffectorPos.y(),
                endEffectorPos.z());

  m_modelRobot.setSampledEndEffectorPos(m_modelState, e);

  // get pelvis pose from message
  gazebo::msgs::Pose pelvisPose = _req.pelvis_pose();

  SimTK::Quaternion q(pelvisPose.orientation().w(),
                      pelvisPose.orientation().x(),
                      pelvisPose.orientation().y(),
                      pelvisPose.orientation().z());
  SimTK::Vec3 v(pelvisPose.position().x(),
                pelvisPose.position().y(),
                pelvisPose.position().z());

  SimTK::Transform X_GP(SimTK::Rotation(q), v);

  // set pelvis pose in m_modelRobot
  m_modelRobot.setSampledPelvisPose(m_modelState, X_GP);
  m_modelRobot.getGravity()
     .setDownDirection(m_modelState, ~X_GP.R()*UnitVec3(-ZAxis));

  // update state
  m_modelRobot.realize(m_modelState, Stage::Velocity);

  // update viz
  // m_viz->report(m_modelState);

  // compute joint torques
  // joint are in the m_modelRobot order
  const Vector& tau = m_modelTasks.getValue(m_modelState);

  // publish joint torques
  _res.set_name("response");
  _res.clear_torques();
  // _res.add_torques(0); // for the world_joint
  for (unsigned int i = 0; i < uIndex.size(); ++i)
  {
    // std::cout << i << " : " << tau[i] << "\n";
    _res.add_torques(tau[uIndex[i]]);
  }
  _result = true;
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  std::cout << "starting responder\n";

  node.reset(new ignition::transport::Node());
  // initialize model state
  m_modelRobot.initialize(m_modelState);

  // visualize
  // m_viz = new Visualizer(m_modelRobot);
  // m_viz->report(m_modelState);

  node->Advertise("/control_request", callback);

  int c = 0;
  // Busy wait loop...replace with your own code as needed.
  while(c != 'q')
  {
    std::cout << "press: g to toggle gravity\n"
              << "       t to toggle task (end effector tracking)\n"
              << "       p to toggle pose control\n>";
    c = getchar();
    if (c == 'g')
    {
      m_modelTasks.toggleGravityComp();
    }
    else if (c == 't')
    {
      m_modelTasks.toggleTask();
    }
    else if (c == 'p')
    {
      m_modelTasks.togglePoseControl();
    }
    // print out status
    std::cout << "-------------------------------\n";
    if (m_modelTasks.isGravityCompensationOn())
      std::cout << "Gravity compensation is now on.\n";
    else
      std::cout << "Gravity compensation is now off.\n";
    if (m_modelTasks.isTaskPointFollowingOn())
      std::cout << "Task point following is now on.\n";
    else
      std::cout << "Task point following is now off.\n";
    if (m_modelTasks.isPoseControlOn())
      std::cout << "Pose control is now on.\n";
    else
      std::cout << "Pose control is now off.\n";
    std::cout << "-------------------------------\n";
  }
  std::cout << "exit!";
}
