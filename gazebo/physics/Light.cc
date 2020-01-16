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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/LightState.hh"
#include "gazebo/physics/Light.hh"


/// \brief Private data for the Light class
class gazebo::physics::LightPrivate
{
  /// \brief Light message container.
  public: msgs::Light msg;

  /// \brief Flag to indicate if light world pose is dirty or not.
  public: bool worldPoseDirty = false;
};

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Light::Light(BasePtr _parent)
  : Entity(_parent), dataPtr(new LightPrivate)
{
  this->AddType(LIGHT);
}

//////////////////////////////////////////////////
Light::~Light()
{
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

  this->dataPtr->msg.MergeFrom(_msg);
}

//////////////////////////////////////////////////
void Light::FillMsg(msgs::Light &_msg)
{
  _msg.MergeFrom(this->dataPtr->msg);

  _msg.set_id(this->GetId());
  _msg.set_name(this->GetScopedName());

  ignition::math::Pose3d pose = this->RelativePose();
  msgs::Set(_msg.mutable_pose(), pose);
}

//////////////////////////////////////////////////
void Light::SetState(const LightState &_state)
{
  if (this->worldPose == _state.Pose())
    return;

  this->worldPose = _state.Pose();
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
const ignition::math::Pose3d &Light::WorldPose() const
{
  EntityPtr parentEnt = boost::dynamic_pointer_cast<Entity>(this->parent);
  if (this->dataPtr->worldPoseDirty && parentEnt)
  {
    this->worldPose = this->InitialRelativePose() +
                      parentEnt->WorldPose();
    this->dataPtr->worldPoseDirty = false;
  }

  return this->worldPose;
}

/////////////////////////////////////////////////
void Light::SetWorldPoseDirty()
{
  // Tell the light object that the next call to ::WorldPose should
  // compute a new worldPose value.
  this->dataPtr->worldPoseDirty = true;
}
