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

#include "ignition/common/Console.hh"
#include "ignition/common/Exception.hh"

#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyScrewJoint::SimbodyScrewJoint(SimTK::MultibodySystem * /*_world*/,
                                     BasePtr _parent)
    : ScrewJoint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyScrewJoint::~SimbodyScrewJoint()
{
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<SimbodyJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::Init()
{
  ignerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetVelocity(unsigned int /*_index*/) const
{
  ignerr << "Not implemented in simbody\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  ignerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetAxis(unsigned int /*_index*/,
    const ignition::math::Vector3 &/*_axis*/)
{
  ignerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetThreadPitch(unsigned int /*_index*/,
    double /*_threadPitch*/)
{
  ignerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetForceImpl(unsigned int /*_index*/, double /*_force*/)
{
  ignerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetHighStop(unsigned int /*_index*/,
  const ignition::math::Angle &/*_angle*/)
{
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetLowStop(unsigned int /*_index*/,
  const ignition::math::Angle &/*_angle*/)
{
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyScrewJoint::GetHighStop(unsigned int /*_index*/)
{
  return ignition::math::Angle();
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyScrewJoint::GetLowStop(unsigned int /*_index*/)
{
  return ignition::math::Angle();
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetMaxForce(unsigned int /*_index*/, double /*_force*/)
{
  ignerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetMaxForce(unsigned int /*index*/)
{
  ignerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
ignition::math::Vector3 SimbodyScrewJoint::GetGlobalAxis(
    unsigned int /*_index*/) const
{
  ignerr << "SimbodyScrewJoint::GetGlobalAxis not implemented\n";
  return ignition::math::Vector3();
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyScrewJoint::GetAngleImpl(
    unsigned int /*_index*/) const
{
  ignerr << "SimbodyScrewJoint::GetAngleImpl not implemented\n";
  return ignition::math::Angle();
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetThreadPitch(unsigned int /*_index*/)
{
  ignerr << "Not implemented in Simbody\n";
  return 0;
}
