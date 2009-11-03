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
/* Desc: A ball joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: BulletBallJoint.cc 7039 2008-09-24 18:06:29Z natepak $
 */

#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "BulletBody.hh"
#include "BulletBallJoint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
BulletBallJoint::BulletBallJoint(btDynamicsWorld *world)
    : BallJoint<BulletJoint>()
{
  this->world = world;
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
BulletBallJoint::~BulletBallJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
// Get the joints anchor point
Vector3 BulletBallJoint::GetAnchor(int index) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////////////////////////////////
// Set the joints anchor point
void BulletBallJoint::SetAnchor(int index, const Vector3 &anchor)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
/// Attach the two bodies with this joint
void BulletBallJoint::Attach( Body *one, Body *two )
{
  BallJoint<BulletJoint>::Attach(one,two);
  BulletBody *bulletBody1 = dynamic_cast<BulletBody*>(this->body1);
  BulletBody *bulletBody2 = dynamic_cast<BulletBody*>(this->body2);

  if (!bulletBody1 || !bulletBody2)
    gzthrow("Requires bullet bodies");

  btRigidBody *rigidBody1 = bulletBody1->GetBulletBody();
  btRigidBody *rigidBody2 = bulletBody2->GetBulletBody();

  Vector3 pivotA, pivotB;

  // Compute the pivot point, based on the anchorPos
  pivotA = this->anchorPos - this->body1->GetAbsPose().pos;
  pivotB = this->anchorPos - this->body2->GetAbsPose().pos;

  this->constraint = new btPoint2PointConstraint( *rigidBody1, *rigidBody2,
      btVector3(pivotA.x, pivotA.y, pivotA.z),
      btVector3(pivotB.x, pivotB.y, pivotB.z)); 

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

