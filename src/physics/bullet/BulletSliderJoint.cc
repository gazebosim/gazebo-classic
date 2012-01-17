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


#include "common/Exception.hh"
#include "common/Console.hh"
#include "BulletLink.hh"
#include "common/XMLConfig.hh"
#include "BulletSliderJoint.hh"

using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


//////////////////////////////////////////////////
// Constructor
BulletSliderJoint::BulletSliderJoint(btDynamicsWorld *_world)
    : SliderJoint<BulletJoint>()
{
  this->world = _world;
}


//////////////////////////////////////////////////
// Destructor
BulletSliderJoint::~BulletSliderJoint()
{
}

//////////////////////////////////////////////////
/// Load the joint
void BulletSliderJoint::Load(common::XMLConfigNode *_node)
{
  SliderJoint<BulletJoint>::Load(_node);
}

//////////////////////////////////////////////////
/// Attach the two bodies with this joint
void BulletSliderJoint::Attach(Link *_one, Link *_two)
{
  SliderJoint<BulletJoint>::Attach(_one, _two);
  BulletLink *bulletLink1 = dynamic_cast<BulletLink*>(this->body1);
  BulletLink *bulletLink2 = dynamic_cast<BulletLink*>(this->body2);

  if (!bulletLink1 || !bulletLink2)
    gzthrow("Requires bullet bodies");

  btRigidLink *rigidLink1 = bulletLink1->GetBulletLink();
  btRigidLink *rigidLink2 = bulletLink2->GetBulletLink();

  btmath::Vector3 anchor, axis1, axis2;
  btTransform frame1, frame2;
  frame1 = btTransform::getIdentity();
  frame2 = btTransform::getIdentity();

  this->constraint = new btSliderConstraint(*rigidLink1, *rigidLink2,
      frame1, frame2, true);

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////
// Get the axis of rotation
math::Vector3 BulletSliderJoint::GetAxis(int _index) const
{
  return **this->axisP;
}

//////////////////////////////////////////////////
// Get the position of the joint
math::Angle BulletSliderJoint::GetAngle(int _index) const
{
  return ((btSliderConstraint*)this->constraint)->getLinearPos();
}

//////////////////////////////////////////////////
// Get the rate of change
double BulletSliderJoint::GetVelocity(int _index) const
{
  gzerr << "Not implemented in bullet\n";
  return 0;
}

//////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void BulletSliderJoint::SetVelocity(int _index, double _angle)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
// Set the axis of motion
void BulletSliderJoint::SetAxis(int _index, const math::Vector3 &_axis)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
// Set the joint damping
void BulletSliderJoint::SetDamping(int /*index*/, const double _damping)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
// Set the slider force
void BulletSliderJoint::SetForce(int _index, double _force)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
/// Set the high stop of an axis(index).
void BulletSliderJoint::SetHighStop(int _index, math::Angle _angle)
{
  ((btSliderConstraint*)this->constraint)->setUpperLinLimit(_angle.GetAsRadian());
}

//////////////////////////////////////////////////
/// Set the low stop of an axis(index).
void BulletSliderJoint::SetLowStop(int _index, math::Angle _angle)
{
  ((btSliderConstraint*)this->constraint)->setLowerLinLimit(_angle.GetAsRadian());
}

//////////////////////////////////////////////////
///  Get the high stop of an axis(index).
math::Angle BulletSliderJoint::GetHighStop(int _index)
{
  return ((btSliderConstraint*)this->constraint)->getUpperLinLimit();
}

//////////////////////////////////////////////////
///  Get the low stop of an axis(index).
math::Angle BulletSliderJoint::GetLowStop(int _index)
{
  return ((btSliderConstraint*)this->constraint)->getLowerLinLimit();
}

//////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void BulletSliderJoint::SetMaxForce(int _/*t*/*index*_/*t*/, double _/*t*/*t*_/*t*_/*t*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double BulletSliderJoint::GetMaxForce(int /*index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}



