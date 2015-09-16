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

// #include <tbb/parallel_for.h>
// #include <tbb/blocked_range.h>
// #include <float.h>
//
// #include <boost/thread/recursive_mutex.hpp>
// #include <sstream>
//
// #include "gazebo/util/OpenAL.hh"
// #include "gazebo/common/KeyFrame.hh"
// #include "gazebo/common/Animation.hh"
// #include "gazebo/common/Plugin.hh"
// #include "gazebo/common/Events.hh"
// #include "gazebo/common/Exception.hh"
// #include "gazebo/common/Console.hh"
// #include "gazebo/common/CommonTypes.hh"
//
// #include "gazebo/physics/Gripper.hh"
// #include "gazebo/physics/Joint.hh"
// #include "gazebo/physics/JointController.hh"
// #include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
// #include "gazebo/physics/PhysicsEngine.hh"
// #include "gazebo/physics/Light.hh"
// #include "gazebo/physics/Contact.hh"
//
// #include "gazebo/sensors/SensorManager.hh"
//
// #include "gazebo/transport/Node.hh"

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
    this->SetWorldPose(msgs::ConvertIgn(_msg.pose()));
}

//////////////////////////////////////////////////
void Light::SetState(const LightState &_state)
{
  this->SetWorldPose(_state.Pose(), true);
}

