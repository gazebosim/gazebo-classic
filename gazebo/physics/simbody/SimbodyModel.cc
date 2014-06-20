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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Gripper.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/physics/simbody/SimbodyModel.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyModel::SimbodyModel(BasePtr _parent)
  : Model(_parent)
{
}

//////////////////////////////////////////////////
SimbodyModel::~SimbodyModel()
{
}

//////////////////////////////////////////////////
void SimbodyModel::Load(sdf::ElementPtr _sdf)
{
  Model::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyModel::Init()
{
  // Record the model's initial pose (for reseting)
  this->SetInitialRelativePose(this->GetWorldPose());

  this->SetRelativePose(this->GetWorldPose());

  // Initialize the bodies before the joints
  for (Base_V::iterator iter = this->children.begin();
       iter!= this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::LINK))
      boost::static_pointer_cast<Link>(*iter)->Init();
    else if ((*iter)->HasType(Base::MODEL))
      boost::static_pointer_cast<SimbodyModel>(*iter)->Init();
  }

  for (unsigned int i = 0; i < this->GetGripperCount(); ++i)
  {
    this->GetGripper(i)->Init();
  }

  // rebuild simbody state
  // this needs to happen before this->joints are used
  physics::SimbodyPhysicsPtr simbodyPhysics =
    boost::dynamic_pointer_cast<physics::SimbodyPhysics>(
      this->GetWorld()->GetPhysicsEngine());
  if (simbodyPhysics)
    simbodyPhysics->InitModel(
        boost::static_pointer_cast<Model>(shared_from_this()));

  // Initialize the joints last.
  Joint_V myJoints = this->GetJoints();
  for (Joint_V::iterator iter = myJoints.begin();
       iter != myJoints.end(); ++iter)
  {
    try
    {
      (*iter)->Init();
    }
    catch(...)
    {
      gzerr << "Init joint failed" << std::endl;
      return;
    }
  }

  // Initialize the joints messages for visualizer
  for (Joint_V::iterator iter = myJoints.begin();
       iter != myJoints.end(); ++iter)
  {
    // The following message used to be filled and sent in Model::LoadJoint
    // It is moved here, after Joint::Init, so that the joint properties
    // can be included in the message.
    msgs::Joint msg;
    (*iter)->FillMsg(msg);
    this->jointPub->Publish(msg);
  }
}

//////////////////////////////////////////////////
// void SimbodyModel::FillMsg(msgs::Model &_msg)
// {
//   // rebuild simbody state
//   // this needs to happen before this->joints are used
//   physics::SimbodyPhysicsPtr simbodyPhysics =
//     boost::dynamic_pointer_cast<physics::SimbodyPhysics>(
//       this->GetWorld()->GetPhysicsEngine());
//   if (simbodyPhysics)
//     simbodyPhysics->InitModel(this);
//
//   Model::FillMsg(_msg);
// }
