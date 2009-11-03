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
/* Desc: A universal joint
 * Author: Nate Keonig
 * Date: 24 May 2009
 * SVN: $Id:$
 */

#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "BulletBody.hh"
#include "BulletUniversalJoint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
BulletUniversalJoint::BulletUniversalJoint(btDynamicsWorld *world )
    : UniversalJoint<BulletJoint>()
{
  this->world = world;
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
BulletUniversalJoint::~BulletUniversalJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Attach the two bodies with this joint
void BulletUniversalJoint::Attach( Body *one, Body *two )
{
  UniversalJoint<BulletJoint>::Attach(one,two);

  BulletBody *bulletBody1 = dynamic_cast<BulletBody*>(this->body1);
  BulletBody *bulletBody2 = dynamic_cast<BulletBody*>(this->body2);

  if (!bulletBody1 || !bulletBody2)
    gzthrow("Requires bullet bodies");

  btRigidBody *rigidBody1 = bulletBody1->GetBulletBody();
  btRigidBody *rigidBody2 = bulletBody2->GetBulletBody();

  btVector3 anchor, axis1, axis2;

  anchor = btVector3(this->anchorPos.x, this->anchorPos.y, this->anchorPos.z);
  axis1 = btVector3((**this->axis1P).x,(**this->axis1P).y,(**this->axis1P).z);
  axis2 = btVector3((**this->axis2P).x,(**this->axis2P).y,(**this->axis2P).z);

  this->constraint = new btUniversalConstraint( *rigidBody1, *rigidBody2,
      anchor, axis1, axis2);

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////////////////////////////////
// Get the anchor point
Vector3 BulletUniversalJoint::GetAnchor(int /*index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void BulletUniversalJoint::SetAnchor( int index, const Vector3 &anchor )
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
// Get the first axis of rotation
Vector3 BulletUniversalJoint::GetAxis(int index) const
{
  btVector3 axis = ((btUniversalConstraint*)this->constraint)->getAxis(index);
  return Vector3(axis.getX(), axis.getY(), axis.getZ());
}

//////////////////////////////////////////////////////////////////////////////
// Set the first axis of rotation
void BulletUniversalJoint::SetAxis( int index, const Vector3 &axis )
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
// Get the angle of an axis 
Angle BulletUniversalJoint::GetAngle(int index) const
{
  if (index == 0)
    return ((btUniversalConstraint*)this->constraint)->getAngle1();
  else
    return ((btUniversalConstraint*)this->constraint)->getAngle2();
}

//////////////////////////////////////////////////////////////////////////////
// Get the angular rate of an axis
double BulletUniversalJoint::GetVelocity(int index) const
{
  gzerr(0) << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void BulletUniversalJoint::SetVelocity(int index, double angle)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
// Set the torque of this joint
void BulletUniversalJoint::SetForce(int index, double torque)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void BulletUniversalJoint::SetMaxForce(int index, double t)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double BulletUniversalJoint::GetMaxForce(int index)
{
  gzerr(0) << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the high stop of an axis(index).
void BulletUniversalJoint::SetHighStop(int index, Angle angle)
{
  if (this->constraint)
    if (index == 0)
      ((btUniversalConstraint*)this->constraint)->setUpperLimit(
        angle.GetAsRadian(), this->GetHighStop(1).GetAsRadian() );
    else
      ((btUniversalConstraint*)this->constraint)->setUpperLimit(
        this->GetHighStop(0).GetAsRadian(), angle.GetAsRadian() );
  else
    gzthrow("Joint must be created first");

}

//////////////////////////////////////////////////////////////////////////////
/// Set the low stop of an axis(index).
void BulletUniversalJoint::SetLowStop(int index, Angle angle)
{
  if (this->constraint)
    if (index == 0)
      ((btUniversalConstraint*)this->constraint)->setLowerLimit(
        angle.GetAsRadian(), this->GetLowStop(1).GetAsRadian() );
    else
      ((btUniversalConstraint*)this->constraint)->setUpperLimit(
        this->GetLowStop(0).GetAsRadian(), angle.GetAsRadian() );
  else
    gzthrow("Joint must be created first");
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the high stop of an axis(index).
Angle BulletUniversalJoint::GetHighStop(int index)
{
  Angle result;

  if (this->constraint)
  {
    btRotationalLimitMotor *motor;
    motor = ((btUniversalConstraint*)this->constraint)->getRotationalLimitMotor(index);

    return motor->m_hiLimit;
  }
  else
    gzthrow("Joint must be created first");

  return result;
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the low stop of an axis(index).
Angle BulletUniversalJoint::GetLowStop(int index)
{
  Angle result;

  if (this->constraint)
  {
    btRotationalLimitMotor *motor;
    motor = ((btUniversalConstraint*)this->constraint)->getRotationalLimitMotor(index);

    return motor->m_loLimit;
  }
  else
    gzthrow("Joint must be created first");

  return result;
}
