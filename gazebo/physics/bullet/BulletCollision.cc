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
/* Desc: BulletCollision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include "physics/bullet/bullet_inc.h"
#include "physics/bullet/BulletCollision.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletCollision::BulletCollision(LinkPtr _parent)
    : Collision(_parent)
{
  this->SetName("Bullet_Collision");
  this->collisionShape = NULL;
}

//////////////////////////////////////////////////
BulletCollision::~BulletCollision()
{
  /*
  delete this->collisionShape;
  this->collisionShape = NULL;
  */
}

//////////////////////////////////////////////////
void BulletCollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletCollision::OnPoseChange()
{
  /*
  math::Pose pose = this->GetRelativePose();
  BulletLink *bbody = static_cast<BulletLink*>(this->body);

  bbody->SetCollisionRelativePose(this, pose);
  */
}

//////////////////////////////////////////////////
void BulletCollision::SetCategoryBits(unsigned int /*_bits*/)
{
}

//////////////////////////////////////////////////
void BulletCollision::SetCollideBits(unsigned int /*_bits*/)
{
}

//////////////////////////////////////////////////
/*Mass BulletCollision::GetLinkMassMatrix()
{
  Mass result;
  return result;
}*/

//////////////////////////////////////////////////
math::Box BulletCollision::GetBoundingBox() const
{
  math::Box result;
  if (this->collisionShape)
  {
    btVector3 btMin, btMax;
    this->collisionShape->getAabb(btTransform::getIdentity(), btMin, btMax);

    result.min.Set(btMin.x(), btMin.y(), btMin.z());
    result.max.Set(btMax.x(), btMax.y(), btMax.z());
  }
  return result;
}

//////////////////////////////////////////////////
void BulletCollision::SetCollisionShape(btCollisionShapePtr _shape)
{
  this->collisionShape = _shape;

  // btmath::Vector3 vec;
  // this->collisionShape->calculateLocalInertia(this->mass.GetAsDouble(), vec);

  // this->mass.SetCoG(this->GetRelativePose().pos);
}

//////////////////////////////////////////////////
btCollisionShapePtr BulletCollision::GetCollisionShape() const
{
  return this->collisionShape;
}

//////////////////////////////////////////////////
void BulletCollision::SetCompoundShapeIndex(int /*_index*/)
{
  // this->compoundShapeIndex = 0;
}
