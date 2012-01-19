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
 */
#include "common/Exception.hh"
#include "common/Console.hh"
#include "BulletJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
BulletJoint::BulletJoint()
  : Joint()
{
  this->constraint = NULL;
}

//////////////////////////////////////////////////
BulletJoint::~BulletJoint()
{
  delete this->constraint;
}

//////////////////////////////////////////////////
void BulletJoint::Load(common::XMLConfigNode *_node)
{
  Joint::Load(_node);
}

//////////////////////////////////////////////////
Link *BulletJoint::GetJointLink(int index) const
{
  Link *result = NULL;

  if (this->constraint == NULL)
    gzthrow("Attach bodies to the joint first");

  if (index == 0 || index == 1)
  {
    BulletLink *bulletLink1 = dynamic_cast<BulletLink*>(this->body1);
    BulletLink *bulletLink2 = dynamic_cast<BulletLink*>(this->body2);
    btRigidLink rigidLink = this->constraint->getRigidLinkA();

    if (bulletLink1 && rigidLink.getUserPointer() == bulletLink1)
      result = this->body1;
    else if (bulletLink2)
      result = this->body2;
  }

  return result;
}


//////////////////////////////////////////////////
bool BulletJoint::AreConnected(Link *_one, Link *_two) const
{
  return this->constraint && ((this->body1 == _one && this->body2 == _two) ||
         (this->body1 == _two && this->body2 == _one));
}

//////////////////////////////////////////////////
void BulletJoint::Detach()
{
  this->body1 = NULL;
  this->body2 = NULL;

  delete this->constraint;
}

//////////////////////////////////////////////////
void BulletJoint::SetERP(double _newERP)
{
}

//////////////////////////////////////////////////
double BulletJoint::GetERP()
{
  return 0;
}

//////////////////////////////////////////////////
void BulletJoint::SetCFM(double _newCFM)
{
}

//////////////////////////////////////////////////
double BulletJoint::GetCFM()
{
  return 0;
}


