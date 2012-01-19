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
#include "BulletSliderJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
BulletSliderJoint::BulletSliderJoint(btDynamicsWorld *_world)
    : SliderJoint<BulletJoint>()
{
  this->world = _world;
}


//////////////////////////////////////////////////
BulletSliderJoint::~BulletSliderJoint()
{
}

//////////////////////////////////////////////////
void BulletSliderJoint::Load(common::XMLConfigNode *_node)
{
  SliderJoint<BulletJoint>::Load(_node);
}

//////////////////////////////////////////////////
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
math::Vector3 BulletSliderJoint::GetAxis(int _index) const
{
  return **this->axisP;
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetAngle(int _index) const
{
  return static_cast<btSliderConstraint*>(this->constraint)->getLinearPos();
}

//////////////////////////////////////////////////
double BulletSliderJoint::GetVelocity(int _index) const
{
  gzerr << "Not implemented in bullet\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetVelocity(int _index, double _angle)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetAxis(int _index, const math::Vector3 &_axis)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetDamping(int /*index*/, const double _damping)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetForce(int _index, double _force)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetHighStop(int _index, math::Angle _angle)
{
  static_cast<btSliderConstraint*>(this->constraint)->setUpperLinLimit(
    _angle.GetAsRadian());
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetLowStop(int _index, math::Angle _angle)
{
  static_cast<btSliderConstraint*>(this->constraint)->setLowerLinLimit(
    _angle.GetAsRadian());
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetHighStop(int _index)
{
  return static_cast<btSliderConstraint*>(this->constraint)->getUpperLinLimit();
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetLowStop(int _index)
{
  return static_cast<btSliderConstraint*>(
      this->constraint)->getLowerLinLimit();
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetMaxForce(int _index, double _force)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double BulletSliderJoint::GetMaxForce(int /*index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}




