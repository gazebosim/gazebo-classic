/*
 * Copyright 2012 Open Source Robotics Foundation
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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
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
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
}

/////////////////////////////////////////////////
void JointTrajectoryPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr /*_sdf*/)
{
  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;

  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&JointTrajectoryPlugin::UpdateStates, this));
}

/////////////////////////////////////////////////
void JointTrajectoryPlugin::FixLink(physics::LinkPtr link)
{
  this->joint = this->world->GetPhysicsEngine()->CreateJoint("revolute",
      this->model);

  this->joint->SetModel(this->model);
  math::Pose pose = link->GetWorldPose();
  // math::Pose  pose(math::Vector3(0, 0, 0.2), math::Quaternion(1, 0, 0, 0));
  this->joint->Load(physics::LinkPtr(), link, pose);
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
void JointTrajectoryPlugin::UpdateStates()
{
  common::Time cur_time = this->world->GetSimTime();

  bool is_paused = this->world->IsPaused();
  if (!is_paused) this->world->SetPaused(true);

  std::map<std::string, double> joint_position_map;
  joint_position_map["arm_shoulder_pan_joint"] = cos(cur_time.Double());
  joint_position_map["arm_elbow_pan_joint"] = -cos(cur_time.Double());
  joint_position_map["arm_wrist_lift_joint"] = -0.35
    + 0.45*cos(0.5*cur_time.Double());
  joint_position_map["arm_wrist_roll_joint"] = -2.9*cos(3.0*cur_time.Double());

  this->model->SetJointPositions(joint_position_map);

  // resume original pause-state
  this->world->SetPaused(is_paused);
}

GZ_REGISTER_MODEL_PLUGIN(JointTrajectoryPlugin)
}
