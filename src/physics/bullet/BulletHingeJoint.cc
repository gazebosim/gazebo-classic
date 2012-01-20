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
/* Desc: A BulletHingeJoint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 */
/*
#include "Model.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "World.hh"
#include "BulletPhysics.hh"
#include "BulletHingeJoint.hh"
*/

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
BulletHingeJoint::BulletHingeJoint(btDynamicsWorld *_world)
    : HingeJoint<BulletJoint>()
{
  this->world = _world;
}


//////////////////////////////////////////////////
BulletHingeJoint::~BulletHingeJoint()
{
}

//////////////////////////////////////////////////
void BulletHingeJoint::Load(common::XMLConfigNode *_node)
{
  HingeJoint<BulletJoint>::Load(_node);
}


//////////////////////////////////////////////////
void BulletHingeJoint::Attach(Link *_one, Link *_two)
{
  HingeJoint<BulletJoint>::Attach(_one, _two);
  BulletLink *bulletLink1 = dynamic_cast<BulletLink*>(this->body1);
  BulletLink *bulletLink2 = dynamic_cast<BulletLink*>(this->body2);

  if (!bulletLink1 || !bulletLink2)
    gzthrow("Requires bullet bodies");

  btRigidLink *rigidLink1 = bulletLink1->GetBulletLink();
  btRigidLink *rigidLink2 = bulletLink2->GetBulletLink();

  math::Vector3 pivotA, pivotB;
  btmath::Vector3 axisA, axisB;

  // Compute the pivot point, based on the anchorPos
  pivotA = (this->anchorPos - this->body1->GetWorldPose().pos);
  pivotB = (this->anchorPos - this->body2->GetWorldPose().pos);

  axisA =
    btmath::Vector3((**this->axisP).x, (**this->axisP).y, (**this->axisP).z);
  axisB =
    btmath::Vector3((**this->axisP).x, (**this->axisP).y, (**this->axisP).z);

  this->constraint = new btHingeConstraint(*rigidLink1, *rigidLink2,
      btmath::Vector3(pivotA.x, pivotA.y, pivotA.z),
      btmath::Vector3(pivotB.x, pivotB.y, pivotB.z), axisA, axisB);

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
  static_cast<btHingeConstraint*>(this->constraint)->setAngularOnly(true);
}

//////////////////////////////////////////////////
math::Vector3 BulletHingeJoint::GetAnchor(int _index) const
{
  btTransform trans =
    static_cast<btHingeConstraint*>(this->constraint)->getAFrame();
  trans.getOrigin() +=
    this->constraint->getRigidLinkA().getCenterOfMassTransform().getOrigin();
  return math::Vector3(trans.getOrigin().getX(),
      trans.getOrigin().getY(), trans.getOrigin().getZ());
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetAnchor(int _index, const math::Vector3 &_anchor)
{
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
math::Vector3 BulletHingeJoint::GetAxis(int _index) const
{
  return (**this->axisP);
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetAxis(int _index, const math::Vector3 &_axis)
{
  gzerr << "Bullet handles setAxis improperly\n";
  // Bullet seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btmath::Vector3 vec(_axis.x, _axis.y, _axis.z);
  ((btHingeConstraint*)this->constraint)->setAxis(vec);
  */
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetDamping(int /*index*/, const double _damping)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
math::Angle BulletHingeJoint::GetAngle(int _index) const
{
  if (this->constraint)
    return ((btHingeConstraint*)this->constraint)->getHingemath::Angle();
  else
    gzthrow("Joint has not been created");
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetVelocity(int _index, double _angle)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double BulletHingeJoint::GetVelocity(int _index) const
{
  gzerr << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetMaxForce(int _index, double _t)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double BulletHingeJoint::GetMaxForce(int _index)
{
  gzerr << "Not implemented\n";
  return 0;
}


//////////////////////////////////////////////////
void BulletHingeJoint::SetForce(int _index, double _torque)
{
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
double BulletHingeJoint::GetForce(int _index)
{
  gzerr << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetHighStop(int _index, math::Angle _angle)
{
  if (this->constraint)
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    static_cast<btHingeConstraint*>(this->constraint)->setLimit(
      this->GetLowStop(_index).GetAsRadian(), _angle.GetAsRadian());
  else
    gzthrow("Joint must be created first");
}

//////////////////////////////////////////////////
void BulletHingeJoint::SetLowStop(int _index, math::Angle _angle)
{
  if (this->constraint)
    // this function has additional parameters that we may one day
    // implement. Be warned that this function will reset them to default
    // settings
    static_cast<btHingeConstraint*>(this->constraint)->setLimit(
        _angle.GetAsRadian(), this->GetHighStop(_index).GetAsRadian());
  else
    gzthrow("Joint must be created first");
}

//////////////////////////////////////////////////
math::Angle BulletHingeJoint::GetHighStop(int _index)
{
  if (this->constraint)
    return static_cast<btHingeConstraint*>(this->constraint)->getUpperLimit();
  else
    gzthrow("Joint must be created first");
}

//////////////////////////////////////////////////
math::Angle BulletHingeJoint::GetLowStop(int _index)
{
  if (this->constraint)
    return static_cast<btHingeConstraint*>(this->constraint)->getLowerLimit();
  else
    gzthrow("Joint must be created first");
}


