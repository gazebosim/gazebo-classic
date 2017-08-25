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
void Light::ProcessMsg(const msgs::Light &_msg)
{
  // Get leaf name
  std::string lightName = _msg.name();
  size_t idx = lightName.rfind("::");
  if (idx != std::string::npos)
    lightName = lightName.substr(idx+2);

  this->SetName(lightName);
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

  // TODO change to RelativePose once lights can be attached to links
  // in link.proto and on the rendering side
  // ignition::math::Pose3d pose = this->GetRelativePose().Ign();
  ignition::math::Pose3d pose = this->GetWorldPose().Ign();
  msgs::Set(_msg.mutable_pose(), pose);
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

/////////////////////////////////////////////////
const math::Pose &Light::GetWorldPose() const
{
  // If true, compute a new world pose value.
  // if (this->worldPoseDirty)
  EntityPtr parentEnt = boost::dynamic_pointer_cast<Entity>(this->parent);
  if (!this->worldPose.IsFinite() && parentEnt)
  {
    this->worldPose = this->GetInitialRelativePose() +
                      parentEnt->GetWorldPose();
    // this->worldPoseDirty = false;
  }

  return this->worldPose;
}

/////////////////////////////////////////////////
void Light::SetWorldPoseDirty()
{
  // Tell the light object that the next call to ::GetWorldPose should
  // compute a new worldPose value.

  // TODO add and use worldPoseDirty member variable
  // instead of making the pose infinite. It was done to avoid breaking ABI
  // this->worldPoseDirty = true;
  double v = std::numeric_limits<double>::infinity();
  this->worldPose.pos.Set(v, v, v);

  /// TODO The following line is added as a workaround to update light pose on
  /// the rendering side without breaking the API/ABI. Later we should update
  /// link.proto and add a repeated light field (breaks ABI) and in rendering we
  /// just attach the light scene node to the parent link node.
  this->PublishPose();
}
