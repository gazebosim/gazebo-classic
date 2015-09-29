/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif


#include "gazebo/physics/World.hh"
#include "gazebo/physics/LightState.hh"
#include "gazebo/physics/Light.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Light::Light(BasePtr _parent)
  : Entity(_parent)
{
  this->AddType(LIGHT);
}

//////////////////////////////////////////////////
void Light::Init()
{
  // Record the light's initial pose (for resetting)
  ignition::math::Pose3d initPose =
      this->sdf->Get<ignition::math::Pose3d>("pose");
  this->SetInitialRelativePose(initPose);
  this->SetRelativePose(initPose);
}

//////////////////////////////////////////////////
void Light::FillMsg(msgs::Light &_msg)
{
  ignition::math::Pose3d relPose = this->GetRelativePose().Ign();

  _msg.set_name(this->GetScopedName());
  msgs::Set(_msg.mutable_pose(), relPose);
}

//////////////////////////////////////////////////
void Light::ProcessMsg(const msgs::Light &_msg)
{
/*
  if (_msg.has_id() && _msg.id() != this->GetId())
  {
    gzerr << "Incorrect ID[" << _msg.id() << " != " << this->GetId() << "]\n";
    return;
  }
  else if ((_msg.has_id() && _msg.id() != this->GetId()) &&
            _msg.name() != this->GetScopedName())
  {
    gzerr << "Incorrect name[" << _msg.name() << " != " << this->GetName()
      << "]\n";
    return;
  }
*/
  this->SetName(this->world->StripWorldName(_msg.name()));
  if (_msg.has_pose())
  {
    this->worldPose = msgs::ConvertIgn(_msg.pose());
  }

//  this->msg->CopyFrom(_msg);
}

//////////////////////////////////////////////////
void Light::SetState(const LightState &_state)
{
  this->worldPose = math::Pose(_state.Pose());
  this->PublishPose();
}

//////////////////////////////////////////////////
void Light::OnPoseChange()
{
/*
  ignition::math::Pose3d p;
  for (unsigned int i = 0; i < this->attachedModels.size(); ++i)
  {
    p = this->GetWorldPose();
    p.pos += this->attachedModelsOffset[i].pos;
    p.rot = p.rot * this->attachedModelsOffset[i].rot;

    this->attachedModels[i]->SetWorldPose(p, true);
  }
*/
}

//////////////////////////////////////////////////
void Light::PublishPose()
{
  this->world->PublishLightPose(boost::dynamic_pointer_cast<Light>(
      shared_from_this()));
}


