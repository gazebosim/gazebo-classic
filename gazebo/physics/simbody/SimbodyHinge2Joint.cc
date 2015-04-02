/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyHinge2Joint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyHinge2Joint::SimbodyHinge2Joint(SimTK::MultibodySystem * /*_world*/,
                                       BasePtr _parent)
    : Hinge2Joint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyHinge2Joint::~SimbodyHinge2Joint()
{
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::Load(sdf::ElementPtr _sdf)
{
  Hinge2Joint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHinge2Joint::GetAnchor(unsigned int /*index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHinge2Joint::GetAxis(unsigned int /*index*/) const
{
  gzerr << "Not implemented";
  return math::Vector3();
}

//////////////////////////////////////////////////
double SimbodyHinge2Joint::GetVelocity(unsigned int /*_index*/) const
{
  gzerr << "Not implemented";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetVelocity(unsigned int /*_index*/,
    double /*_angle*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetAxis(unsigned int /*_index*/,
    const math::Vector3 &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetForceImpl(
    unsigned int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
double SimbodyHinge2Joint::GetMaxForce(unsigned int /*_index*/)
{
  gzerr << "Not implemented";
  return 0;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHinge2Joint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzerr << "SimbodyHinge2Joint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodyHinge2Joint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzerr << "SimbodyHinge2Joint::GetAngleImpl not implemented\n";
  return math::Angle();
}
