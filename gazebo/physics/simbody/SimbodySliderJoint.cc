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
/* Desc: A simbody slider or primastic joint
 * Author: Nate Koenig
 * Date: 13 Oct 2009
 */

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodySliderJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodySliderJoint::SimbodySliderJoint(SimTK::MultibodySystem *_world,
                                       BasePtr _parent)
    : SliderJoint<SimbodyJoint>(_parent)
{
}

//////////////////////////////////////////////////
SimbodySliderJoint::~SimbodySliderJoint()
{
}

//////////////////////////////////////////////////
void SimbodySliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodySliderJoint::Attach(LinkPtr _one, LinkPtr _two)
{
  SliderJoint<SimbodyJoint>::Attach(_one, _two);

  SimbodyLinkPtr simbodyChildLink =
    boost::shared_static_cast<SimbodyLink>(this->childLink);
  SimbodyLinkPtr simbodyParentLink =
    boost::shared_static_cast<SimbodyLink>(this->parentLink);

  if (!simbodyChildLink || !simbodyParentLink)
    gzthrow("Requires simbody bodies");

  // Add the joint to the world

  // Allows access to impulse
}

//////////////////////////////////////////////////
math::Angle SimbodySliderJoint::GetAngle(int /*_index*/) const
{
  return math::Angle();
}

//////////////////////////////////////////////////
double SimbodySliderJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented in simbody\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetVelocity(int /*_index*/, double _angle)
{
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzdbg << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetDamping(int /*index*/, const double _damping)
{
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetForce(int /*_index*/, double _force)
{
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetHighStop(int /*_index*/,
                                    const math::Angle &/*_angle*/)
{
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetLowStop(int /*_index*/,
                                   const math::Angle &/*_angle*/)
{
}

//////////////////////////////////////////////////
math::Angle SimbodySliderJoint::GetHighStop(int /*_index*/)
{
  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle SimbodySliderJoint::GetLowStop(int /*_index*/)
{
  return math::Angle();
}

//////////////////////////////////////////////////
void SimbodySliderJoint::SetMaxForce(int /*_index*/, double _force)
{
}

//////////////////////////////////////////////////
double SimbodySliderJoint::GetMaxForce(int /*_index*/)
{
  return math::Angle().Radian();
}

//////////////////////////////////////////////////
math::Vector3 SimbodySliderJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodySliderJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return math::Angle();
}


