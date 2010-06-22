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
/* Desc: A bullet slider or primastic joint
 * Author: Nate Keonig
 * Date: 13 Oct 2009
 * SVN: $Id:$
 */


#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "BulletBody.hh"
#include "XMLConfig.hh"
#include "BulletSliderJoint.hh"

using namespace gazebo;

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


