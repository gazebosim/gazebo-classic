/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Inertial.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/ShapePrivate.hh"

#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletSphereShape.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
BulletSphereShape::BulletSphereShape(CollisionPtr _parent)
: SphereShape(_parent)
{
}

/////////////////////////////////////////////////
BulletSphereShape::~BulletSphereShape()
{
}

/////////////////////////////////////////////////
void BulletSphereShape::SetRadius(double _radius)
{
  if (_radius < 0)
  {
    gzerr << "Sphere shape does not support negative radius\n";
    return;
  }
  if (ignition::math::equal(_radius, 0.0))
  {
    // Warn user, but still create shape with very small value
    // otherwise later resize operations using setLocalScaling
    // will not be possible
    gzwarn << "Setting sphere shape's radius to zero \n";
    _radius = 1e-4;
  }

  SphereShape::SetRadius(_radius);
  BulletCollisionPtr bParent;
  bParent = std::dynamic_pointer_cast<BulletCollision>(
      this->shapeDPtr->collisionParent);

  btCollisionShape *shape = bParent->CollisionShape();
  if (!shape)
  {
    this->initialSize =
      ignition::math::Vector3d(_radius, _radius, _radius);
    bParent->SetCollisionShape(new btSphereShape(_radius));
  }
  else
  {
    btVector3 sphereScale;
    sphereScale.setX(_radius / this->initialSize.X());
    sphereScale.setY(_radius / this->initialSize.Y());
    sphereScale.setZ(_radius / this->initialSize.Z());

    shape->setLocalScaling(sphereScale);

    // clear bullet cache and re-add the collision shape
    // otherwise collisions won't work properly after scaling
    BulletLinkPtr bLink =
      std::dynamic_pointer_cast<BulletLink>(
          bParent->Link());
    bLink->ClearCollisionCache();

    // remove and add the shape again
    if (bLink->BtLink()->getCollisionShape()->isCompound())
    {
      btCompoundShape *compoundShape =
        dynamic_cast<btCompoundShape *>(
            bLink->BtLink()->getCollisionShape());

      compoundShape->removeChildShape(shape);
      ignition::math::Pose3d relativePose =
        this->shapeDPtr->collisionParent->RelativePose();
      relativePose.Pos() -= bLink->Inertia().CoG();
      compoundShape->addChildShape(
          BulletTypes::ConvertPose(relativePose), shape);
    }
  }
}
