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
/* Desc: The base Bullet joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 15 May 2009
 * SVN: $Id$
 */

#include "BulletBody.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "BulletJoint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
BulletJoint::BulletJoint()
  : Joint()
{
  this->constraint = NULL;
}

//////////////////////////////////////////////////////////////////////////////
// Desctructor
BulletJoint::~BulletJoint()
{
  if (this->constraint)
    delete this->constraint;
}

//////////////////////////////////////////////////////////////////////////////
// Load a joint
void BulletJoint::Load(XMLConfigNode *node)
{
  Joint::Load(node);

}

//////////////////////////////////////////////////////////////////////////////
// Get the body to which the joint is attached according the _index
Body *BulletJoint::GetJointBody( int index ) const
{
  Body *result=NULL;

  if (this->constraint == NULL)
    gzthrow("Attach bodies to the joint first");

  if (index == 0 || index ==1)
  {
    BulletBody *bulletBody1 = dynamic_cast<BulletBody*>(this->body1);
    BulletBody *bulletBody2 = dynamic_cast<BulletBody*>(this->body2);
    btRigidBody rigidBody = this->constraint->getRigidBodyA();

    if (bulletBody1 && rigidBody.getUserPointer() == bulletBody1)
      result = this->body1;
    else if (bulletBody2)
      result = this->body2;
  }

  return result;
}


//////////////////////////////////////////////////////////////////////////////
// Determines of the two bodies are connected by a joint
bool BulletJoint::AreConnected( Body *one, Body *two ) const
{
  return this->constraint && ((this->body1 == one && this->body2 == two) || 
         (this->body1 == two && this->body2 == one));
}

//////////////////////////////////////////////////////////////////////////////
// Detach this joint from all bodies
void BulletJoint::Detach()
{
  this->body1 = NULL;
  this->body2 = NULL;

  if (this->constraint)
    delete this->constraint;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the ERP of this joint
void BulletJoint::SetERP(double newERP)
{
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ERP of this joint
double BulletJoint::GetERP()
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the CFM of this joint
void BulletJoint::SetCFM(double newCFM)
{
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ERP of this joint
double BulletJoint::GetCFM()
{
  return 0;
}
