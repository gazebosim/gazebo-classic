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
/*
 * Desc: a test for setting joint angles
 * Author: John Hsu
 */

#include <plugins/JointTrajectoryPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
JointTrajectoryPlugin::JointTrajectoryPlugin()
{
}

/////////////////////////////////////////////////
JointTrajectoryPlugin::~JointTrajectoryPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void JointTrajectoryPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr /*_sdf*/)
{
  // Get the world name.
  this->model = _parent;
  this->world = this->model->GetWorld();

  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));

  for (physics::Joint_V::const_iterator j = this->model->GetJoints().begin();
                        j != this->model->GetJoints().end(); ++j)
    (*j)->SetAngle(0, 0);

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&JointTrajectoryPlugin::UpdateStates, this, _1));
}

/////////////////////////////////////////////////
void JointTrajectoryPlugin::FixLink(physics::LinkPtr _link)
{
  this->joint = this->world->GetPhysicsEngine()->CreateJoint("revolute",
      this->model);

  this->joint->SetModel(this->model);
  math::Pose pose = _link->GetWorldPose();
  // math::Pose  pose(math::Vector3(0, 0, 0.2), math::Quaternion(1, 0, 0, 0));
  this->joint->Load(physics::LinkPtr(), _link, pose);
  this->joint->SetAxis(0, math::Vector3(0, 0, 0));
  this->joint->SetHighStop(0, 0);
  this->joint->SetLowStop(0, 0);
  this->joint->SetAnchor(0, pose.pos);
  this->joint->Init();
}

/////////////////////////////////////////////////
void JointTrajectoryPlugin::UnfixLink()
{
  this->joint.reset();
}

/////////////////////////////////////////////////
void JointTrajectoryPlugin::UpdateStates(const common::UpdateInfo & /*_info*/)
{
  common::Time cur_time = this->world->GetSimTime();

  // for (physics::Joint_V::const_iterator j = this->model->GetJoints().begin();
  //                       j != this->model->GetJoints().end(); ++j)
  //   gzerr << cur_time << " " << (*j)->GetScopedName() << "\n";

  bool is_paused = this->world->IsPaused();
  if (!is_paused) this->world->SetPaused(true);

  std::map<std::string, double> joint_position_map;
  joint_position_map["simple_arm_gripper::simple_arm::arm_shoulder_pan_joint"]
    = cos(cur_time.Double());
  joint_position_map["simple_arm_gripper::simple_arm::arm_elbow_pan_joint"]
    = -cos(cur_time.Double());
  joint_position_map["simple_arm_gripper::simple_arm::arm_wrist_lift_joint"]
    = -0.35 + 0.45*cos(0.5*cur_time.Double());
  joint_position_map["simple_arm_gripper::simple_arm::arm_wrist_roll_joint"]
    = -2.9*cos(3.0*cur_time.Double());

  this->model->SetJointPositions(joint_position_map);

  // resume original pause-state
  this->world->SetPaused(is_paused);
}

GZ_REGISTER_MODEL_PLUGIN(JointTrajectoryPlugin)
}
