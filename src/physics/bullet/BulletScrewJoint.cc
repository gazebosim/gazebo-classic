/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
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
/* Desc: A bullet screw or primastic joint
 * Author: Nate Keonig
 * Date: 13 Oct 2009
 */

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/bullet/BulletBody.hh"
#include "physics/bullet/BulletPhysics.hh"
#include "physics/bullet/BulletTypes.hh"
#include "physics/bullet/BulletScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletScrewJoint::BulletScrewJoint(btDynamicsWorld *_world)
    : ScrewJoint<BulletJoint>()
{
  this->world = _world;
}

//////////////////////////////////////////////////
BulletScrewJoint::~BulletScrewJoint()
{
}

//////////////////////////////////////////////////
void BulletScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<BulletJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletScrewJoint::Attach(BodyPtr _one, BodyPtr _two)
{
  ScrewJoint<BulletJoint>::Attach(_one, _two);

  BulletBodyPtr bulletChildBody =
    boost::shared_static_cast<BulletBody>(this->childBody);
  BulletBodyPtr bulletParentBody =
    boost::shared_static_cast<BulletBody>(this->parentBody);

  if (!bulletChildBody || !bulletParentBody)
    gzthrow("Requires bullet bodies");

  btTransform frame1, frame2;
  frame1 = btTransform::getIdentity();
  frame2 = btTransform::getIdentity();

  math::Vector3 pivotA, pivotB;

  pivotA = this->anchorPos - this->parentBody->GetWorldPose().pos;
  pivotB = this->anchorPos - this->childBody->GetWorldPose().pos;

  pivotA = this->parentBody->GetWorldPose().rot.RotateVectorReverse(pivotA);
  pivotB = this->childBody->GetWorldPose().rot.RotateVectorReverse(pivotB);

  frame1.setOrigin(btVector3(pivotA.x, pivotA.y, pivotA.z));
  frame2.setOrigin(btVector3(pivotB.x, pivotB.y, pivotB.z));

  frame1.getBasis().setEulerZYX(0, M_PI*0.5, 0);
  frame2.getBasis().setEulerZYX(0, M_PI*0.5, 0);

  this->btScrew = new btSliderConstraint(
      *bulletChildBody->GetBulletBody(),
      *bulletParentBody->GetBulletBody(),
      frame2, frame1, true);

  this->constraint = this->btScrew;

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetAngle(int /*_index*/) const
{
  return this->btScrew->getLinearPos();
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented in bullet\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetForce(int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetHighStop(int /*_index*/, math::Angle _angle)
{
  this->btScrew->setUpperLinLimit(_angle.GetAsRadian());
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetLowStop(int /*_index*/, math::Angle _angle)
{
  this->btScrew->setLowerLinLimit(_angle.GetAsRadian());
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetHighStop(int /*_index*/)
{
  return this->btScrew->getUpperLinLimit();
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetLowStop(int /*_index*/)
{
  return this->btScrew->getLowerLinLimit();
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetMaxForce(int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetMaxForce(int /*index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
math::Vector3 BulletScrewJoint::GetGlobalAxis(int /*_index*/) const
{
  gzerr << "BulletScrewJoint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetAngleImpl(int /*_index*/) const
{
  gzerr << "BulletScrewJoint::GetAngleImpl not implemented\n";
  return math::Angle();
}
