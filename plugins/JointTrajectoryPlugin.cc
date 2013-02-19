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
                                 sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;

  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  // Get Joints
  sdf::ElementPtr jointElem = _sdf->GetElement("joint");
  while (jointElem)
  {
    // FIXME: below segfaults on second <joint></joint> entry, SDF problem?
    gzerr << jointElem->GetValueString() << "\n";

    physics::JointPtr j = this->model->GetJoint(jointElem->GetValueString());
    if (j)
    {
      this->joints.push_back(j);
    }
    else
    {
      j.reset();
    }
    jointElem = _sdf->GetNextElement("joint");
  }

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

  for (unsigned int i = 0; i < this->joints.size(); ++i)
    this->joints[i]->SetAngle(0, cos(cur_time.Double()));
}

GZ_REGISTER_MODEL_PLUGIN(JointTrajectoryPlugin)
}
