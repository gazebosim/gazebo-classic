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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: BulletHinge2Joint.cc 7129 2008-11-12 19:38:15Z natepak $
 */

#include "common/Exception.hh"
#include "common/Console.hh"
#include "BulletPhysics.hh"
#include "BulletHinge2Joint.hh"

using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


//////////////////////////////////////////////////
BulletHinge2Joint::BulletHinge2Joint(btDynamicsWorld *_world)
    : Hinge2Joint<BulletJoint>()
{
  this->world = _world;
}


//////////////////////////////////////////////////
BulletHinge2Joint::~BulletHinge2Joint()
{
}

//////////////////////////////////////////////////
void BulletHinge2Joint::Load(common::XMLConfigNode *_node)
{
  Hinge2Joint<BulletJoint>::Load(_node);
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SaveJoint(std::string &_prefix, std::ostream &_stream)
{
  Hinge2Joint<BulletJoint>::SaveJoint(_prefix, _stream);
}

//////////////////////////////////////////////////
void BulletHinge2Joint::Attach(Link *_one, Link *_two)
{
  Hinge2Joint<BulletJoint>::Attach(_one, _two);
  BulletLink *bulletLink1 = dynamic_cast<BulletLink*>(this->body1);
  BulletLink *bulletLink2 = dynamic_cast<BulletLink*>(this->body2);

  if (!bulletLink1 || !bulletLink2)
    gzthrow("Requires bullet bodies");

  btRigidLink *rigidLink1 = bulletLink1->GetBulletLink();
  btRigidLink *rigidLink2 = bulletLink2->GetBulletLink();

  btmath::Vector3 anchor, axis1, axis2;

  anchor = btmath::Vector3(this->anchorPos.x, this->anchorPos.y,
                           this->anchorPos.z);
  axis1 = btmath::Vector3((**this->axis1P).x, (**this->axis1P).y,
                          (**this->axis1P).z);
  axis2 = btmath::Vector3((**this->axis2P).x, (**this->axis2P).y,
                          (**this->axis2P).z);

  this->constraint = new btHinge2Constraint(*rigidLink1, *rigidLink2,
      anchor, axis1, axis2);

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////
math::Vector3 BulletHinge2Joint::GetAnchor(int /*index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
math::Vector3 BulletHinge2Joint::GetAxis(int /*index*/) const
{
  btmath::Vector3 vec = ((btHinge2Constraint*)this->constraint)->getAxis1();
  return math::Vector3(vec.getX(), vec.getY(), vec.getZ());
}

//////////////////////////////////////////////////
math::Angle BulletHinge2Joint::GetAngle(int _index) const
{
  return ((btHinge2Constraint*)this->constraint)->getmath::Angle1();
}

//////////////////////////////////////////////////
double BulletHinge2Joint::GetVelocity(int _index) const
{
  gzerr << "Not implemented";
  return 0;
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetVelocity(int _index, double _angle)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetAnchor(int _index, const math::Vector3 &_anchor)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetAxis(int _index, const math::Vector3 &_axis)
{
  gzerr << "Not implemented";
}


//////////////////////////////////////////////////
void BulletHinge2Joint::SetDamping(int /*index*/, const double _damping)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetForce(int _index, double _torque)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetMaxForce(int _index, double _t)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
double BulletHinge2Joint::GetMaxForce(int _index)
{
  gzerr << "Not implemented";
  return 0;
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetHighStop(int _index, math::Angle _angle)
{
  static_cast<btHinge2Constraint*>(this->constraint)->setUpperLimit(
      _angle.GetAsRadian());
}

//////////////////////////////////////////////////
void BulletHinge2Joint::SetLowStop(int _index, math::Angle _angle)
{
  static_cast<btHinge2Constraint*>(this->constraint)->setLowerLimit(
      _angle.GetAsRadian());
}

//////////////////////////////////////////////////
math::Angle BulletHinge2Joint::GetHighStop(int _index)
{
  btRotationalLimitMotor *motor =
    static_cast<btHinge2Constraint*>(this->constraint)->getRotationalLimitMotor(
        _index);
  if (motor)
    return motor->m_hiLimit;

  gzthrow("Unable to get high stop for axis _index[" << _index << "]");
  return 0;
}

//////////////////////////////////////////////////
math::Angle BulletHinge2Joint::GetLowStop(int _index)
{
  btRotationalLimitMotor *motor =
    static_cast<btHinge2Constraint*>(this->constraint)->getRotationalLimitMotor(
        _index);
  if (motor)
    return motor->m_loLimit;

  gzthrow("Unable to get high stop for axis _index[" << _index << "]");
  return 0;
}


