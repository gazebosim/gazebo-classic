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
/* Desc: A universal joint
 * Author: Nate Koenig
 * Date: 24 May 2009
 */

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyUniversalJoint::SimbodyUniversalJoint(SimTK::MultibodySystem *_world,
  BasePtr _parent) : UniversalJoint<SimbodyJoint>(_parent)
{
}

//////////////////////////////////////////////////
SimbodyUniversalJoint::~SimbodyUniversalJoint()
{
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::Init()
{
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
math::Vector3 SimbodyUniversalJoint::GetAnchor(int /*index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetAnchor(int /*_index*/,
                                     const math::Vector3 &/*_anchor*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
math::Vector3 SimbodyUniversalJoint::GetAxis(int _index) const
{
  return math::Vector3();
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetAxis(int /*_index*/,
                                   const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
math::Angle SimbodyUniversalJoint::GetAngle(int _index) const
{
  return math::Angle();
}

//////////////////////////////////////////////////
double SimbodyUniversalJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetForce(int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double SimbodyUniversalJoint::GetMaxForce(int /*_index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetHighStop(int _index, const math::Angle &_angle)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetLowStop(int _index, const math::Angle &_angle)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
math::Angle SimbodyUniversalJoint::GetHighStop(int _index)
{
  math::Angle result;

  return result;
}

//////////////////////////////////////////////////
math::Angle SimbodyUniversalJoint::GetLowStop(int _index)
{
  math::Angle result;

  return result;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyUniversalJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "SimbodyUniversalJoint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodyUniversalJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "SimbodyUniversalJoint::GetAngleImpl not implemented\n";
  return math::Angle();
}
