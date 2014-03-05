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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

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
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented in simbody\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetThreadPitch(int /*_index*/, double /*_threadPitch*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetForceImpl(int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetHighStop(int /*_index*/,
  const math::Angle &/*_angle*/)
{
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetLowStop(int /*_index*/,
  const math::Angle &/*_angle*/)
{
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetHighStop(int /*_index*/)
{
  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetLowStop(int /*_index*/)
{
  return math::Angle();
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetMaxForce(int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetMaxForce(int /*index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyScrewJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "SimbodyScrewJoint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "SimbodyScrewJoint::GetAngleImpl not implemented\n";
  return math::Angle();
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetThreadPitch(unsigned int /*_index*/)
{
  gzerr << "Not implemented in Simbody\n";
  return 0;
}
