/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/physics/PlaneShape.hh"

#include "gazebo/physics/bullet/bullet_inc.h"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletSurfaceParams.hh"
#include "gazebo/physics/bullet/BulletCollisionPrivate.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletCollision::BulletCollision(LinkPtr _parent)
: Collision(*new BulletCollisionPrivate, _parent),
  bulletCollisionDPtr(static_cast<BulletCollisionPrivate*>(this->collDPtr))
{
  this->SetName("Bullet_Collision");
  this->bulletCollisionDPtr->collisionShape = nullptr;
  this->bulletCollisionDPtr->surface.reset(new BulletSurfaceParams());
}

//////////////////////////////////////////////////
BulletCollision::~BulletCollision()
{
}

//////////////////////////////////////////////////
void BulletCollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }
}

//////////////////////////////////////////////////
void BulletCollision::OnPoseChange()
{
  // math::Pose pose = this->GetRelativePose();
  // BulletLinkPtr bbody =
  //     boost::dynamic_pointer_cast<BulletLink>(this->parent);

  // bbody->motionState.setWorldTransform(this, pose);
}

//////////////////////////////////////////////////
void BulletCollision::SetCategoryBits(const unsigned int _bits)
{
  this->bulletCollisionDPtr->categoryBits = _bits;
}

//////////////////////////////////////////////////
void BulletCollision::SetCollideBits(const unsigned int _bits)
{
  this->bulletCollisionDPtr->collideBits = _bits;
}

//////////////////////////////////////////////////
unsigned int BulletCollision::CategoryBits() const
{
  return this->bulletCollisionDPtr->categoryBits;
}

//////////////////////////////////////////////////
unsigned int BulletCollision::CollideBits() const
{
  return this->bulletCollisionDPtr->collideBits;
}

//////////////////////////////////////////////////
/*Mass BulletCollision::GetLinkMassMatrix()
{
  Mass result;
  return result;
}*/

//////////////////////////////////////////////////
ignition::math::Box BulletCollision::BoundingBox() const
{
  ignition::math::Box result;
  if (this->bulletCollisionDPtr->collisionShape)
  {
    btVector3 btMin, btMax;
    this->bulletCollisionDPtr->collisionShape->getAabb(
        btTransform::getIdentity(), btMin, btMax);

    result = math::Box(math::Vector3(btMin.x(), btMin.y(), btMin.z()),
                       math::Vector3(btMax.x(), btMax.y(), btMax.z()));

    if (this->ShapeType() & PLANE_SHAPE)
    {
      PlaneShapePtr plane =
        std::dynamic_pointer_cast<PlaneShape>(this->bulletCollisionDPtr->shape);
      ignition::math::Vector3d normal = plane->Normal();
      if (normal == ignition::math::Vector3d::UnitZ)
      {
        // Should check altitude, but it's not implemented
        result.Max().Z(0.0);
      }
    }
  }
  return result;
}

//////////////////////////////////////////////////
void BulletCollision::SetCollisionShape(btCollisionShape *_shape,
    const bool _placeable)
{
  Collision::SetCollision(_placeable);
  this->bulletCollisionDPtr->collisionShape = _shape;

  // btmath::Vector3 vec;
  // this->bulletCollisionDPtr->collisionShape->calculateLocalInertia(
  // this->mass.GetAsDouble(), vec);

  // this->mass.SetCoG(this->GetRelativePose().pos);

  // this->bulletCollisionDPtr->collisionShape->setFriction(1.0);
  // this->bulletCollisionDPtr->collisionShape->setAnisotropicFriction(
  // btVector3(0, 0, 0));
}

//////////////////////////////////////////////////
btCollisionShape *BulletCollision::CollisionShape() const
{
  return this->bulletCollisionDPtr->collisionShape;
}

//////////////////////////////////////////////////
void BulletCollision::SetCompoundShapeIndex(const int /*_index*/)
{
  // this->compoundShapeIndex = 0;
}

/////////////////////////////////////////////////
BulletSurfaceParamsPtr BulletCollision::BulletSurface() const
{
  return std::dynamic_pointer_cast<BulletSurfaceParams>(
      this->bulletCollisionDPtr->surface);
}
