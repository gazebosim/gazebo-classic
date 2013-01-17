/*
 * Copyright 2012 Open Source Robotics Foundation
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
 * Author: Nate Koenig, Andrew Howard
 * Date: 15 May 2009
 */

#include "common/Exception.hh"
#include "common/Console.hh"

#include "physics/bullet/bullet_inc.h"
#include "physics/bullet/BulletLink.hh"
#include "physics/bullet/BulletJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletJoint::BulletJoint(BasePtr _parent)
  : Joint(_parent)
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
LinkPtr BulletJoint::GetJointLink(int _index) const
{
  LinkPtr result;

  if (this->constraint == NULL)
    gzthrow("Attach bodies to the joint first");

  if (_index == 0 || _index == 1)
  {
    BulletLinkPtr bulletLink1 =
      boost::shared_static_cast<BulletLink>(this->childLink);

    BulletLinkPtr bulletLink2 =
      boost::shared_static_cast<BulletLink>(this->parentLink);

    btRigidBody rigidLink = this->constraint->getRigidBodyA();

    if (bulletLink1 && rigidLink.getUserPointer() == bulletLink1.get())
      result = this->childLink;
    else if (bulletLink2)
      result = this->parentLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool BulletJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  return this->constraint && ((this->childLink.get() == _one.get() &&
                               this->parentLink.get() == _two.get()) ||
                              (this->childLink.get() == _two.get() &&
                               this->parentLink.get() == _one.get()));
}

//////////////////////////////////////////////////
void BulletJoint::Detach()
{
  this->childLink.reset();
  this->parentLink.reset();

  delete this->constraint;
}

//////////////////////////////////////////////////
JointWrench BulletJoint::GetForceTorque(int /*_index*/)
{
  JointWrench wrench;
  return wrench;
}
