/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"

#include "gazebo/physics/PlaneShape.hh"

#include "gazebo/physics/bullet/bullet_inc.h"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletSurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletCollision::BulletCollision(LinkPtr _parent)
    : Collision(_parent)
{
  this->SetName("Bullet_Collision");
  this->collisionShape = nullptr;
  this->surface.reset(new BulletSurfaceParams());
  this->collideBits = GZ_ALL_COLLIDE;
}

//////////////////////////////////////////////////
BulletCollision::~BulletCollision()
{
  /*
  delete this->collisionShape;
  this->collisionShape = nullptr;
  */
}

//////////////////////////////////////////////////
void BulletCollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);

  if (this->IsStatic())
  {
    this->collideBits = ~GZ_FIXED_COLLIDE;
  }
  else
  {
    this->collideBits = this->GetSurface()->collideBitmask;
  }
}

//////////////////////////////////////////////////
void BulletCollision::OnPoseChange()
{
  // auto pose = this->RelativePose();
  // BulletLinkPtr bbody =
  //     boost::dynamic_pointer_cast<BulletLink>(this->parent);

  // bbody->motionState.setWorldTransform(this, pose);
}

//////////////////////////////////////////////////
void BulletCollision::SetCategoryBits(unsigned int _bits)
{
  gzwarn << "SetCategoryBits is not implemented" << std::endl;
  // Prevent unused variable warning
  this->categoryBits = _bits;
}

//////////////////////////////////////////////////
void BulletCollision::SetCollideBits(unsigned int _bits)
{
  // Collide bits apply to the whole rigid body, so all sibling
  // collisions on the same link must have identical collide bits
  auto numChildren = this->link->GetChildCount();
  for (decltype(numChildren) c = 0; c < numChildren; ++c)
  {
    BasePtr child = this->link->GetChild(c);
    if (child->HasType(COLLISION))
    {
      BulletCollision *col = dynamic_cast<BulletCollision *>(&*child);
      col->GetSurface()->collideBitmask = _bits;
    }
  }

  BulletLink *bulletLink = dynamic_cast<BulletLink *>(&*this->parent);

  // Set mask and group because BulletRayShape still uses them, TODO remove
  btRigidBody *bod = bulletLink->GetBulletLink();
  if (nullptr != bod)
  {
    // Need a rigid body in the world before bits on proxy can be changed
    btBroadphaseProxy *proxy = bod->getBroadphaseProxy();
    GZ_ASSERT(nullptr != proxy, "body must be added to world for collide bits");
    proxy->m_collisionFilterMask = _bits;
    proxy->m_collisionFilterGroup = _bits;

    // CollideBits are checked in the overlappingFilterCallback which is
    // only called when objects are added to the world
    bulletLink->RemoveAndAddBody();
  }
}

//////////////////////////////////////////////////
unsigned int BulletCollision::GetCategoryBits() const
{
  gzwarn << "GetCategoryBits is not implemented" << std::endl;
  // Prevent unused variable warning
  return this->categoryBits;
}

//////////////////////////////////////////////////
unsigned int BulletCollision::GetCollideBits() const
{
  return this->GetSurface()->collideBitmask;
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
  if (this->collisionShape)
  {
    btVector3 btMin, btMax;
    btTransform relPose = BulletTypes::ConvertPose(this->RelativePose());
    this->collisionShape->getAabb(relPose, btMin, btMax);

    result = ignition::math::Box(
        ignition::math::Vector3d(btMin.x(), btMin.y(), btMin.z()),
        ignition::math::Vector3d(btMax.x(), btMax.y(), btMax.z()));

    if (this->GetShapeType() & PLANE_SHAPE)
    {
      PlaneShapePtr plane =
        boost::dynamic_pointer_cast<PlaneShape>(this->shape);
      auto normal = plane->Normal();
      if (normal == ignition::math::Vector3d::UnitZ)
      {
        // Should check altitude, but it's not implemented
        result.Max().Z() =  0.0;
      }
    }
  }
  return result;
}

//////////////////////////////////////////////////
void BulletCollision::SetCollisionShape(btCollisionShape *_shape,
    bool _placeable)
{
  Collision::SetPlaceable(_placeable);
  this->collisionShape = _shape;

  // btVector3 vec;
  // this->collisionShape->calculateLocalInertia(this->mass.GetAsDouble(), vec);

  // this->mass.SetCoG(this->GetRelativePose().pos);

  // this->collisionShape->setFriction(1.0);
  // this->collisionShape->setAnisotropicFriction(btVector3(0, 0, 0));
}

//////////////////////////////////////////////////
btCollisionShape *BulletCollision::GetCollisionShape() const
{
  return this->collisionShape;
}

//////////////////////////////////////////////////
void BulletCollision::SetCompoundShapeIndex(int /*_index*/)
{
  // this->compoundShapeIndex = 0;
}

/////////////////////////////////////////////////
BulletSurfaceParamsPtr BulletCollision::GetBulletSurface() const
{
  return boost::dynamic_pointer_cast<BulletSurfaceParams>(this->surface);
}
