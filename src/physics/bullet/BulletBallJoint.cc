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

#include "common/Exception.hh"
#include "common/Console.hh"
#include "physics/bullet/BulletBody.hh"
#include "physics/bullet/BulletBallJoint.hh"

using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


//////////////////////////////////////////////////
// Constructor
BulletBallJoint::BulletBallJoint(btDynamicsWorld *_world)
    : BallJoint<BulletJoint>()
{
  this->world = _world;
}

//////////////////////////////////////////////////
// Destructor
BulletBallJoint::~BulletBallJoint()
{
}

//////////////////////////////////////////////////
// Get the joints anchor point
math::Vector3 BulletBallJoint::GetAnchor(int _index) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
// Set the joints anchor point
void BulletBallJoint::SetAnchor(int _index, const math::Vector3 &_anchor)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
// Set the joint damping
void BulletBallJoint::SetDamping(int /*index*/, const double _damping)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
/// Attach the two bodies with this joint
void BulletBallJoint::Attach(Link *_one, Link *_two)
{
  BallJoint<BulletJoint>::Attach(_one, _two);
  BulletLink *bulletLink1 = dynamic_cast<BulletLink*>(this->body1);
  BulletLink *bulletLink2 = dynamic_cast<BulletLink*>(this->body2);

  if (!bulletLink1 || !bulletLink2)
    gzthrow("Requires bullet bodies");

  btRigidLink *rigidLink1 = bulletLink1->GetBulletLink();
  btRigidLink *rigidLink2 = bulletLink2->GetBulletLink();

  math::Vector3 pivotA, pivotB;

  // Compute the pivot point, based on the anchorPos
  pivotA = this->anchorPos - this->body1->GetWorldPose().pos;
  pivotB = this->anchorPos - this->body2->GetWorldPose().pos;

  this->constraint = new btPoint2PointConstraint(*rigidLink1, *rigidLink2,
      btmath::Vector3(pivotA.x, pivotA.y, pivotA.z),
      btmath::Vector3(pivotB.x, pivotB.y, pivotB.z));

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}



