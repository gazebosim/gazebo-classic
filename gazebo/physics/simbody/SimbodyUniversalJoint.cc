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

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyUniversalJoint::SimbodyUniversalJoint(SimTK::MultibodySystem* /*_world*/,
  BasePtr _parent) : UniversalJoint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyUniversalJoint::~SimbodyUniversalJoint()
{
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::Load(sdf::ElementPtr _sdf)
{
  UniversalJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::Init()
{
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
math::Vector3 SimbodyUniversalJoint::GetAnchor(unsigned int /*_index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyUniversalJoint::GetAxis(unsigned int /*_index*/) const
{
  return math::Vector3();
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetAxis(unsigned int /*_index*/,
                                   const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented\n";
}


//////////////////////////////////////////////////
double SimbodyUniversalJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetVelocity(unsigned int /*_index*/,
    double /*_angle*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetForceImpl(unsigned int /*_index*/,
    double /*_torque*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double SimbodyUniversalJoint::GetMaxForce(unsigned int /*_index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetHighStop(unsigned int /*_index*/,
  const math::Angle &/*_angle*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetLowStop(unsigned int /*_index*/,
  const math::Angle &/*_angle*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
math::Angle SimbodyUniversalJoint::GetHighStop(unsigned int /*_index*/)
{
  math::Angle result;
  gzerr << "Not implemented\n";
  return result;
}

//////////////////////////////////////////////////
math::Angle SimbodyUniversalJoint::GetLowStop(unsigned int /*_index*/)
{
  math::Angle result;
  gzerr << "Not implemented\n";
  return result;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyUniversalJoint::GetGlobalAxis(
    unsigned int /*_index*/) const
{
  gzerr << "SimbodyUniversalJoint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodyUniversalJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzerr << "SimbodyUniversalJoint::GetAngleImpl not implemented\n";
  return math::Angle();
}
