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
/* Desc: The base Bullet joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 15 May 2009
 * SVN: $Id$
 */

#include "BulletBody.hh"
#include "common/GazeboError.hh"
#include "common/GazeboMessage.hh"
#include "BulletJoint.hh"

using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


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
