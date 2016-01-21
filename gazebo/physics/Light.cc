/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
void Light::ProcessMsg(const msgs::Light &_msg)
{
  this->SetName(this->world->StripWorldName(_msg.name()));
  if (_msg.has_pose())
  {
    this->worldPose = msgs::ConvertIgn(_msg.pose());
  }

  this->msg.MergeFrom(_msg);
}

//////////////////////////////////////////////////
void Light::FillMsg(msgs::Light &_msg)
{
  _msg.MergeFrom(this->msg);

  _msg.set_name(this->GetScopedName());

  ignition::math::Pose3d relPose = this->GetRelativePose().Ign();
  msgs::Set(_msg.mutable_pose(), relPose);
}

//////////////////////////////////////////////////
void Light::SetState(const LightState &_state)
{
  if (this->worldPose == math::Pose(_state.Pose()))
    return;

  this->worldPose = math::Pose(_state.Pose());
  this->PublishPose();
}

//////////////////////////////////////////////////
void Light::PublishPose()
{
  this->world->PublishLightPose(boost::dynamic_pointer_cast<Light>(
      shared_from_this()));
}

//////////////////////////////////////////////////
void Light::OnPoseChange()
{
}

