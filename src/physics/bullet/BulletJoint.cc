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

#include "physics/bullet/bullet_inc.h"
#include "physics/bullet/BulletBody.hh"
#include "physics/bullet/BulletJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletJoint::BulletJoint()
  : Joint()
{
  this->constraint = NULL;
  this->world = NULL;
}

//////////////////////////////////////////////////
BulletJoint::~BulletJoint()
{
  delete this->constraint;
  this->world = NULL;
}

//////////////////////////////////////////////////
void BulletJoint::Load(sdf::ElementPtr _sdf)
{
  Joint::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
BodyPtr BulletJoint::GetJointBody(int _index) const
{
  BodyPtr result;

  if (this->constraint == NULL)
    gzthrow("Attach bodies to the joint first");

  if (_index == 0 || _index == 1)
  {
    BulletBodyPtr bulletBody1 =
      boost::shared_static_cast<BulletBody>(this->childBody);

    BulletBodyPtr bulletBody2 =
      boost::shared_static_cast<BulletBody>(this->parentBody);

    btRigidBody rigidBody = this->constraint->getRigidBodyA();

    if (bulletBody1 && rigidBody.getUserPointer() == bulletBody1.get())
      result = this->childBody;
    else if (bulletBody2)
      result = this->parentBody;
  }

  return result;
}

//////////////////////////////////////////////////
bool BulletJoint::AreConnected(BodyPtr _one, BodyPtr _two) const
{
  return this->constraint && ((this->childBody.get() == _one.get() &&
                               this->parentBody.get() == _two.get()) ||
                              (this->childBody.get() == _two.get() &&
                               this->parentBody.get() == _one.get()));
}

//////////////////////////////////////////////////
void BulletJoint::Detach()
{
  this->childBody.reset();
  this->parentBody.reset();

  delete this->constraint;
}
