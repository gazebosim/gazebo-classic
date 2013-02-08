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
/* Desc: A SimbodyHingeJoint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */
#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/simbody/SimbodyLink.hh"
#include "physics/simbody/SimbodyPhysics.hh"
#include "physics/simbody/SimbodyHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyHingeJoint::SimbodyHingeJoint(SimTK::MultibodySystem *_world,
                                     BasePtr _parent)
    : HingeJoint<SimbodyJoint>(_parent)
{
  this->world = _world;
}

//////////////////////////////////////////////////
SimbodyHingeJoint::~SimbodyHingeJoint()
{
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::Attach(LinkPtr _one, LinkPtr _two)
{
  HingeJoint<SimbodyJoint>::Attach(_one, _two);

  SimbodyLinkPtr simbodyChildLink =
    boost::shared_static_cast<SimbodyLink>(this->childLink);
  SimbodyLinkPtr simbodyParentLink =
    boost::shared_static_cast<SimbodyLink>(this->parentLink);

  // if (!simbodyChildLink || !simbodyParentLink)
  //   gzthrow("Requires simbody bodies");

  sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  math::Vector3 axis = axisElem->GetValueVector3("xyz");

  math::Vector3 pivotA, pivotB, axisA, axisB;

  if (this->parentLink)
  {
    // Compute the pivot point, based on the anchorPos
    pivotA = this->anchorPos + this->childLink->GetWorldPose().pos
                             - this->parentLink->GetWorldPose().pos;
    pivotA = this->parentLink->GetWorldPose().rot.RotateVectorReverse(pivotA);
    // Compute axis
    axisA = this->parentLink->GetWorldPose().rot.RotateVectorReverse(axis);
    axisA = axisA.Round();
  }
  if (this->childLink)
  {
    pivotB = this->anchorPos;
    pivotB = this->childLink->GetWorldPose().rot.RotateVectorReverse(pivotB);
    axisB = this->childLink->GetWorldPose().rot.RotateVectorReverse(axis);
    axisB = axisB.Round();
  }

  // Add the joint to the world

  // Allows access to impulse
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHingeJoint::GetAnchor(int /*_index*/) const
{
  gzerr << "Not implemented...\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetAnchor(int /*_index*/,
                                 const math::Vector3 &/*_anchor*/)
{
  gzerr << "Not implemented...\n";
  // The anchor (pivot in Simbody lingo), can only be set on creation
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetAngle(int /*_index*/) const
{
  gzerr << "Not implemented...\n";
  return math::Angle();
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetMaxForce(int /*_index*/, double _t)
{
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetMaxForce(int /*_index*/)
{
  gzerr << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetForce(int /*_index*/, double _torque)
{
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetForce(int /*_index*/)
{
  gzerr << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetHighStop(int /*_index*/,
                                   const math::Angle &/*_angle*/)
{
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetLowStop(int /*_index*/,
                                  const math::Angle &/*_angle*/)
{
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetHighStop(int /*_index*/)
{
  math::Angle result;
  gzerr << "Not implemented...\n";
  return result;
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetLowStop(int /*_index*/)
{
  math::Angle result;
  gzerr << "Not implemented...\n";
  return result;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyHingeJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "SimbodyHingeJoint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodyHingeJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "SimbodyHingeJoint::GetAngleImpl not implemented\n";
  return math::Angle();
}
