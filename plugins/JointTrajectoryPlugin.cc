/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
