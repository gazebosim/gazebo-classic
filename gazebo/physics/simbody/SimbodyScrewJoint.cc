/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: A simbody screw or primastic joint
 * Author: Nate Koenig
 * Date: 13 Oct 2009
 */

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/simbody/SimbodyLink.hh"
#include "physics/simbody/SimbodyPhysics.hh"
#include "physics/simbody/SimbodyTypes.hh"
#include "physics/simbody/SimbodyScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyScrewJoint::SimbodyScrewJoint(btDynamicsWorld *_world, BasePtr _parent)
    : ScrewJoint<SimbodyJoint>(_parent)
{
  this->world = _world;
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
void SimbodyScrewJoint::Attach(LinkPtr _one, LinkPtr _two)
{
  ScrewJoint<SimbodyJoint>::Attach(_one, _two);

  SimbodyLinkPtr simbodyChildLink =
    boost::shared_static_cast<SimbodyLink>(this->childLink);
  SimbodyLinkPtr simbodyParentLink =
    boost::shared_static_cast<SimbodyLink>(this->parentLink);

  if (!simbodyChildLink || !simbodyParentLink)
    gzthrow("Requires simbody bodies");

  btTransform frame1, frame2;
  frame1 = btTransform::getIdentity();
  frame2 = btTransform::getIdentity();

  math::Vector3 pivotA, pivotB;

  pivotA = this->anchorPos - this->parentLink->GetWorldPose().pos;
  pivotB = this->anchorPos - this->childLink->GetWorldPose().pos;

  pivotA = this->parentLink->GetWorldPose().rot.RotateVectorReverse(pivotA);
  pivotB = this->childLink->GetWorldPose().rot.RotateVectorReverse(pivotB);

  frame1.setOrigin(btVector3(pivotA.x, pivotA.y, pivotA.z));
  frame2.setOrigin(btVector3(pivotB.x, pivotB.y, pivotB.z));

  frame1.getBasis().setEulerZYX(0, M_PI*0.5, 0);
  frame2.getBasis().setEulerZYX(0, M_PI*0.5, 0);

  this->btScrew = new btSliderConstraint(
      *simbodyChildLink->GetSimbodyLink(),
      *simbodyParentLink->GetSimbodyLink(),
      frame2, frame1, true);

  this->constraint = this->btScrew;

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetAngle(int /*_index*/) const
{
  return this->btScrew->getLinearPos();
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
void SimbodyScrewJoint::SetForce(int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetHighStop(int /*_index*/, const math::Angle &_angle)
{
  this->btScrew->setUpperLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetLowStop(int /*_index*/, const math::Angle &_angle)
{
  this->btScrew->setLowerLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetHighStop(int /*_index*/)
{
  return this->btScrew->getUpperLinLimit();
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetLowStop(int /*_index*/)
{
  return this->btScrew->getLowerLinLimit();
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
