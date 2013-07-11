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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyHinge2Joint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyHinge2Joint::SimbodyHinge2Joint(SimTK::MultibodySystem *_world,
                                       BasePtr _parent)
    : Hinge2Joint<SimbodyJoint>(_parent)
{
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
void SimbodyHinge2Joint::Attach(LinkPtr _one, LinkPtr _two)
{
  Hinge2Joint<SimbodyJoint>::Attach(_one, _two);

  SimbodyLinkPtr simbodyChildLink =
    boost::shared_static_cast<SimbodyLink>(this->childLink);
  SimbodyLinkPtr simbodyParentLink =
    boost::shared_static_cast<SimbodyLink>(this->parentLink);

  if (!simbodyChildLink || !simbodyParentLink)
    gzthrow("Requires simbody bodies");

  sdf::ElementPtr axis1Elem = this->sdf->GetElement("axis");
  math::Vector3 axis1 = axis1Elem->GetValueVector3("xyz");

  sdf::ElementPtr axis2Elem = this->sdf->GetElement("axis");
  math::Vector3 axis2 = axis2Elem->GetValueVector3("xyz");

  SimTK::Vec3 banchor(this->anchorPos.x, this->anchorPos.y, this->anchorPos.z);
  SimTK::Vec3 baxis1(axis1.x, axis1.y, axis1.z);
  SimTK::Vec3 baxis2(axis2.x, axis2.y, axis2.z);

  // Add the joint to the world

  // Allows access to impulse
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHinge2Joint::GetAnchor(int /*index*/) const
{
  gzerr << "Not implemented";
  return this->anchorPos;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHinge2Joint::GetAxis(int /*index*/) const
{
  gzerr << "Not implemented";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodyHinge2Joint::GetAngle(int /*_index*/) const
{
  gzerr << "Not implemented";
  return math::Angle();
}

//////////////////////////////////////////////////
double SimbodyHinge2Joint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetAnchor(int /*_index*/,
                                  const math::Vector3 &/*_anchor*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetForce(int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetMaxForce(int /*_index*/, double /*_t*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
double SimbodyHinge2Joint::GetMaxForce(int /*_index*/)
{
  gzerr << "Not implemented";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetHighStop(int /*_index*/, const math::Angle &_angle)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void SimbodyHinge2Joint::SetLowStop(int /*_index*/, const math::Angle &_angle)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
math::Angle SimbodyHinge2Joint::GetHighStop(int _index)
{
  gzerr << "Not implemented";
  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle SimbodyHinge2Joint::GetLowStop(int _index)
{
  gzerr << "Not implemented";
  return math::Angle();
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHinge2Joint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "SimbodyHinge2Joint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodyHinge2Joint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "SimbodyHinge2Joint::GetAngleImpl not implemented\n";
  return math::Angle();
}
