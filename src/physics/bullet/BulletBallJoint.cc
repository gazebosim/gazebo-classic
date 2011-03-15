/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: A ball joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: BulletBallJoint.cc 7039 2008-09-24 18:06:29Z natepak $
 */

#include "common/GazeboError.hh"
#include "common/GazeboMessage.hh"
#include "BulletBody.hh"
#include "BulletBallJoint.hh"

using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


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
common::Vector3 BulletBallJoint::GetAnchor(int index) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////////////////////////////////
// Set the joints anchor point
void BulletBallJoint::SetAnchor(int index, const common::Vector3 &anchor)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void BulletBallJoint::SetDamping( int /*index*/, const double damping )
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

  common::Vector3 pivotA, pivotB;

  // Compute the pivot point, based on the anchorPos
  pivotA = this->anchorPos - this->body1->GetWorldPose().pos;
  pivotB = this->anchorPos - this->body2->GetWorldPose().pos;

  this->constraint = new btPoint2PointConstraint( *rigidBody1, *rigidBody2,
      btcommon::Vector3(pivotA.x, pivotA.y, pivotA.z),
      btcommon::Vector3(pivotB.x, pivotB.y, pivotB.z)); 

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

