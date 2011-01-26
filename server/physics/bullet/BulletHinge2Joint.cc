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

#include "Global.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "BulletBody.hh"
#include "XMLConfig.hh"
#include "BulletPhysics.hh"
#include "BulletHinge2Joint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
BulletHinge2Joint::BulletHinge2Joint( btDynamicsWorld *world)
    : Hinge2Joint<BulletJoint>()
{
  this->world = world;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
BulletHinge2Joint::~BulletHinge2Joint()
{
}

//////////////////////////////////////////////////////////////////////////////
///  Load the joint
void BulletHinge2Joint::Load(XMLConfigNode *node)
{
  Hinge2Joint<BulletJoint>::Load(node);
}

//////////////////////////////////////////////////////////////////////////////
/// Save a joint to a stream in XML format
void BulletHinge2Joint::SaveJoint(std::string &prefix, std::ostream &stream)
{
  Hinge2Joint<BulletJoint>::SaveJoint(prefix, stream);
}

//////////////////////////////////////////////////////////////////////////////
/// Attach the two bodies with this joint
void BulletHinge2Joint::Attach( Body *one, Body *two )
{
  Hinge2Joint<BulletJoint>::Attach(one,two);
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

  this->constraint = new btHinge2Constraint( *rigidBody1, *rigidBody2,
      anchor, axis1, axis2); 

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////////////////////////////////
// Get anchor point
Vector3 BulletHinge2Joint::GetAnchor(int /*index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////////////////////////////////
// Get first axis of rotation
Vector3 BulletHinge2Joint::GetAxis(int /*index*/) const
{
  btVector3 vec = ((btHinge2Constraint*)this->constraint)->getAxis1();
  return Vector3(vec.getX(), vec.getY(), vec.getZ());
}

//////////////////////////////////////////////////////////////////////////////
// Get angle of rotation about first axis
Angle BulletHinge2Joint::GetAngle(int index) const
{
  return ((btHinge2Constraint*)this->constraint)->getAngle1();
}

//////////////////////////////////////////////////////////////////////////////
// Get rate of rotation about first axis
double BulletHinge2Joint::GetVelocity(int index) const
{
  gzerr(0) << "Not implemented";
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void BulletHinge2Joint::SetVelocity(int index, double angle)
{
  gzerr(0) << "Not implemented";
}

//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void BulletHinge2Joint::SetAnchor( int index, const Vector3 &anchor )
{
  gzerr(0) << "Not implemented";
}

//////////////////////////////////////////////////////////////////////////////
// Set the first axis of rotation
void BulletHinge2Joint::SetAxis( int index, const Vector3 &axis )
{
  gzerr(0) << "Not implemented";
}


//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void BulletHinge2Joint::SetDamping( int /*index*/, const double damping )
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
// Set torque
void BulletHinge2Joint::SetForce(int index, double torque)
{
  gzerr(0) << "Not implemented";
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void BulletHinge2Joint::SetMaxForce(int index, double t)
{
  gzerr(0) << "Not implemented";
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double BulletHinge2Joint::GetMaxForce(int index)
{
  gzerr(0) << "Not implemented";
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the high stop of an axis(index).
void BulletHinge2Joint::SetHighStop(int index, Angle angle)
{
  ((btHinge2Constraint*)this->constraint)->setUpperLimit(angle.GetAsRadian());
}

//////////////////////////////////////////////////////////////////////////////
/// Set the low stop of an axis(index).
void BulletHinge2Joint::SetLowStop(int index, Angle angle)
{
  ((btHinge2Constraint*)this->constraint)->setLowerLimit(angle.GetAsRadian());
}
 
//////////////////////////////////////////////////////////////////////////////
/// Get the high stop of an axis(index).
Angle BulletHinge2Joint::GetHighStop(int index)
{
  btRotationalLimitMotor *motor = ((btHinge2Constraint*)this->constraint)->getRotationalLimitMotor(index);
  if (motor)
    return motor->m_hiLimit;

  gzthrow("Unable to get high stop for axis index[" << index << "]");
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the low stop of an axis(index).
Angle BulletHinge2Joint::GetLowStop(int index)
{
  btRotationalLimitMotor *motor = ((btHinge2Constraint*)this->constraint)->getRotationalLimitMotor(index);
  if (motor)
    return motor->m_loLimit;

  gzthrow("Unable to get high stop for axis index[" << index << "]");
  return 0;
}
