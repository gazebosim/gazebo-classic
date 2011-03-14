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
/* Desc: A bullet slider or primastic joint
 * Author: Nate Keonig
 * Date: 13 Oct 2009
 * SVN: $Id:$
 */


#include "common/GazeboError.hh"
#include "common/GazeboMessage.hh"
#include "BulletBody.hh"
#include "common/XMLConfig.hh"
#include "BulletSliderJoint.hh"

using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Constructor
BulletSliderJoint::BulletSliderJoint( btDynamicsWorld *world  )
    : SliderJoint<BulletJoint>()
{
  this->world = world;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
BulletSliderJoint::~BulletSliderJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load the joint
void BulletSliderJoint::Load(XMLConfigNode *node)
{
  SliderJoint<BulletJoint>::Load(node);
}

//////////////////////////////////////////////////////////////////////////////
/// Attach the two bodies with this joint
void BulletSliderJoint::Attach( Body *one, Body *two )
{
  SliderJoint<BulletJoint>::Attach(one,two);
  BulletBody *bulletBody1 = dynamic_cast<BulletBody*>(this->body1);
  BulletBody *bulletBody2 = dynamic_cast<BulletBody*>(this->body2);

  if (!bulletBody1 || !bulletBody2)
    gzthrow("Requires bullet bodies");

  btRigidBody *rigidBody1 = bulletBody1->GetBulletBody();
  btRigidBody *rigidBody2 = bulletBody2->GetBulletBody();

  btVector3 anchor, axis1, axis2;
  btTransform frame1, frame2;
  frame1 = btTransform::getIdentity();
  frame2 = btTransform::getIdentity();

  this->constraint = new btSliderConstraint( *rigidBody1, *rigidBody2,
      frame1, frame2, true); 

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
Vector3 BulletSliderJoint::GetAxis(int index) const
{
  return **this->axisP;
}

//////////////////////////////////////////////////////////////////////////////
// Get the position of the joint
Angle BulletSliderJoint::GetAngle(int index) const
{
  return ((btSliderConstraint*)this->constraint)->getLinearPos();
}

//////////////////////////////////////////////////////////////////////////////
// Get the rate of change
double BulletSliderJoint::GetVelocity(int index) const
{
  gzerr(0) << "Not implemented in bullet\n";
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void BulletSliderJoint::SetVelocity(int index, double angle)
{
  gzerr(0) << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////////////////////////////////
// Set the axis of motion
void BulletSliderJoint::SetAxis( int index, const Vector3 &axis )
{
  gzerr(0) << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void BulletSliderJoint::SetDamping( int /*index*/, const double damping )
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
// Set the slider force
void BulletSliderJoint::SetForce(int index, double force)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
/// Set the high stop of an axis(index).
void BulletSliderJoint::SetHighStop(int index, Angle angle)
{
  ((btSliderConstraint*)this->constraint)->setUpperLinLimit(angle.GetAsRadian());
}

//////////////////////////////////////////////////////////////////////////////
/// Set the low stop of an axis(index).
void BulletSliderJoint::SetLowStop(int index, Angle angle)
{
  ((btSliderConstraint*)this->constraint)->setLowerLinLimit(angle.GetAsRadian());
}
 
//////////////////////////////////////////////////////////////////////////////
///  Get the high stop of an axis(index).
Angle BulletSliderJoint::GetHighStop(int index)
{
  return ((btSliderConstraint*)this->constraint)->getUpperLinLimit();
}

//////////////////////////////////////////////////////////////////////////////
///  Get the low stop of an axis(index).
Angle BulletSliderJoint::GetLowStop(int index)
{
  return ((btSliderConstraint*)this->constraint)->getLowerLinLimit();
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void BulletSliderJoint::SetMaxForce(int /*index*/, double /*t*/)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double BulletSliderJoint::GetMaxForce(int /*index*/)
{
  gzerr(0) << "Not implemented\n";
  return 0;
}


