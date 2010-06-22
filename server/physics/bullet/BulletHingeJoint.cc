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
/* Desc: A BulletHingeJoint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: BulletHingeJoint.cc 7640 2009-05-13 02:06:08Z natepak $
 */

#include "Model.hh"
#include "GazeboMessage.hh"
#include "World.hh"
#include "BulletBody.hh"
#include "BulletPhysics.hh"
#include "XMLConfig.hh"
#include "Global.hh"
#include "BulletHingeJoint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
BulletHingeJoint::BulletHingeJoint(btDynamicsWorld *world )
    : HingeJoint<BulletJoint>()
{
  this->world = world;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
BulletHingeJoint::~BulletHingeJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load a hinge joint
void BulletHingeJoint::Load(XMLConfigNode *node)
{
  HingeJoint<BulletJoint>::Load(node);
}


//////////////////////////////////////////////////////////////////////////////
/// Attach the two bodies with this joint
void BulletHingeJoint::Attach( Body *one, Body *two )
{
  HingeJoint<BulletJoint>::Attach(one,two);
  BulletBody *bulletBody1 = dynamic_cast<BulletBody*>(this->body1);
  BulletBody *bulletBody2 = dynamic_cast<BulletBody*>(this->body2);

  if (!bulletBody1 || !bulletBody2)
    gzthrow("Requires bullet bodies");

  btRigidBody *rigidBody1 = bulletBody1->GetBulletBody();
  btRigidBody *rigidBody2 = bulletBody2->GetBulletBody();

  Vector3 pivotA, pivotB;
  btVector3 axisA, axisB;

  // Compute the pivot point, based on the anchorPos
  pivotA = (this->anchorPos - this->body1->GetWorldPose().pos);
  pivotB = (this->anchorPos - this->body2->GetWorldPose().pos);

  axisA = btVector3((**this->axisP).x,(**this->axisP).y,(**this->axisP).z);
  axisB = btVector3((**this->axisP).x,(**this->axisP).y,(**this->axisP).z);

  this->constraint = new btHingeConstraint( *rigidBody1, *rigidBody2,
      btVector3(pivotA.x, pivotA.y, pivotA.z),
      btVector3(pivotB.x, pivotB.y, pivotB.z), axisA, axisB); 

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
  ((btHingeConstraint*)this->constraint)->setAngularOnly(true);
}

//////////////////////////////////////////////////////////////////////////////
// Get the anchor point
Vector3 BulletHingeJoint::GetAnchor(int index ) const
{
  btTransform trans = ((btHingeConstraint*)this->constraint)->getAFrame();
  trans.getOrigin() += this->constraint->getRigidBodyA().getCenterOfMassTransform().getOrigin();
  return Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}

//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void BulletHingeJoint::SetAnchor( int index, const Vector3 &anchor )
{
  gzerr(0) << "Not implemented...\n";
}

//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
Vector3 BulletHingeJoint::GetAxis(int index) const
{
  return (**this->axisP);
}

//////////////////////////////////////////////////////////////////////////////
// Set the axis of rotation
void BulletHingeJoint::SetAxis( int index, const Vector3 &axis )
{
  gzerr(0) << "Bullet handles setAxis improperly\n";
  // Bullet seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btVector3 vec(axis.x, axis.y, axis.z);
  ((btHingeConstraint*)this->constraint)->setAxis(vec);
  */
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void BulletHingeJoint::SetDamping( int /*index*/, const double damping )
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
// Get the angle of rotation
Angle BulletHingeJoint::GetAngle(int index ) const
{
  if (this->constraint)
    return ((btHingeConstraint*)this->constraint)->getHingeAngle();
  else
    gzthrow("Joint has not been created");
}

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void BulletHingeJoint::SetVelocity(int index, double angle)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
// Get the rotation rate
double BulletHingeJoint::GetVelocity(int index) const
{
  gzerr(0) << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void BulletHingeJoint::SetMaxForce(int index, double t)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double BulletHingeJoint::GetMaxForce(int index)
{
  gzerr(0) << "Not implemented\n";
  return 0;
}


//////////////////////////////////////////////////////////////////////////////
// Set the torque of this joint
void BulletHingeJoint::SetForce(int index, double torque)
{
  gzerr(0) << "Not implemented...\n";
}

//////////////////////////////////////////////////////////////////////////////
/// Get the torque of a joint.
double BulletHingeJoint::GetForce(int index)
{
  gzerr(0) << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the high stop of an axis(index).
void BulletHingeJoint::SetHighStop(int index, Angle angle)
{
  if (this->constraint)
    // this function has additional parameters that we may one day 
    // implement. Be warned that this function will reset them to default
    // settings
    ((btHingeConstraint*)this->constraint)->setLimit( 
      this->GetLowStop(index).GetAsRadian(), angle.GetAsRadian() );
  else
    gzthrow("Joint must be created first");
}

//////////////////////////////////////////////////////////////////////////////
/// Set the low stop of an axis(index).
void BulletHingeJoint::SetLowStop(int index, Angle angle)
{
  if (this->constraint)
    // this function has additional parameters that we may one day 
    // implement. Be warned that this function will reset them to default
    // settings
    ((btHingeConstraint*)this->constraint)->setLimit( angle.GetAsRadian(), 
      this->GetHighStop(index).GetAsRadian() );
  else
    gzthrow("Joint must be created first");

}
 
//////////////////////////////////////////////////////////////////////////////
/// Get the high stop of an axis(index).
Angle BulletHingeJoint::GetHighStop(int index)
{
  if (this->constraint)
    return ((btHingeConstraint*)this->constraint)->getUpperLimit();
  else
    gzthrow("Joint must be created first");
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the low stop of an axis(index).
Angle BulletHingeJoint::GetLowStop(int index)
{
  if (this->constraint)
    return ((btHingeConstraint*)this->constraint)->getLowerLimit();
  else
    gzthrow("Joint must be created first");
}
