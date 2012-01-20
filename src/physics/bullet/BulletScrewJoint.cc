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
/* Desc: A bullet screw or primastic joint
 * Author: Nate Keonig
 * Date: 13 Oct 2009
 */


#include "BulletBody.hh"
#include "BulletScrewJoint.hh"

using namespace gazebo;

//////////////////////////////////////////////////
BulletScrewJoint::BulletScrewJoint(btDynamicsWorld *_world)
    : ScrewJoint<BulletJoint>()
{
  this->world = _world;
  gzthrow(std::string("bullet screw constraint is copied from BulletSlider,") +
          " not a screw joint.");
}


//////////////////////////////////////////////////
BulletScrewJoint::~BulletScrewJoint()
{
}

//////////////////////////////////////////////////
void BulletScrewJoint::Load(XMLConfigNode *_node)
{
  ScrewJoint<BulletJoint>::Load(_node);
}

//////////////////////////////////////////////////
void BulletScrewJoint::Attach(Body *_one, Body *_two)
{
  ScrewJoint<BulletJoint>::Attach(_one, _two);
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

  this->constraint = new btSliderConstraint(*rigidBody1, *rigidBody2,
      frame1, frame2, true);

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////
Vector3 BulletScrewJoint::GetAxis(int _index) const
{
  return **this->axisP;
}

//////////////////////////////////////////////////
Angle BulletScrewJoint::GetAngle(int _index) const
{
  return static_cast<btSliderConstraint*>(this->constraint)->getLinearPos();
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetVelocity(int _index) const
{
  gzerr(0) << "Not implemented in bullet\n";
  return 0;
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetVelocity(int _index, double _angle)
{
  gzerr(0) << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetAxis(int _index, const Vector3 &_axis)
{
  gzerr(0) << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetDamping(int /*index*/, const double _damping)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetForce(int _index, double _force)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetHighStop(int _index, Angle _angle)
{
  static_cast<btSliderConstraint*>(this->constraint)->setUpperLinLimit(
    _angle.GetAsRadian());
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetLowStop(int _index, Angle _angle)
{
  static_cast<btSliderConstraint*>(this->constraint)->setLowerLinLimit(
    _angle.GetAsRadian());
}

//////////////////////////////////////////////////
Angle BulletScrewJoint::GetHighStop(int _index)
{
  return static_cast<btSliderConstraint*>(this->constraint)->getUpperLinLimit();
}

//////////////////////////////////////////////////
Angle BulletScrewJoint::GetLowStop(int _index)
{
  return static_cast<btSliderConstraint*>(this->constraint)->getLowerLinLimit();
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetMaxForce(int _index, double _force)
{
  gzerr(0) << "Not implemented\n";
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetMaxForce(int /*index*/)
{
  gzerr(0) << "Not implemented\n";
  return 0;
}




