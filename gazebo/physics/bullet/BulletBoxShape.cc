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
#include <ignition/math/Pose3.hh>

#include "gazebo/common/Console.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Inertial.hh"
#include "gazebo/physics/ShapePrivate.hh"

#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletBoxShape.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
BulletBoxShape::BulletBoxShape(CollisionPtr _parent)
: BoxShape(_parent)
{
}

/////////////////////////////////////////////////
BulletBoxShape::~BulletBoxShape()
{
}

/////////////////////////////////////////////////
void BulletBoxShape::SetSize(const ignition::math::Vector3d &_size)
{
  if (_size.Min() < 0)
  {
    gzerr << "Box shape does not support negative size\n";
    return;
  }
  ignition::math::Vector3d size = _size;
  if (ignition::math::equal(size.X(), 0.0))
  {
    // Warn user, but still create shape with very small value
    // otherwise later resize operations using setLocalScaling
    // will not be possible
    gzwarn << "Setting box shape's x to zero \n";
    size.X(1e-4);
  }
  if (ignition::math::equal(size.Y(), 0.0))
  {
    gzwarn << "Setting box shape's y to zero \n";
    size.Y(1e-4);
  }
  if (ignition::math::equal(size.Z(), 0.0))
  {
    gzwarn << "Setting box shape's z to zero \n";
    size.Z(1e-4);
  }

  BoxShape::SetSize(size);
  BulletCollisionPtr bParent;
  bParent = std::dynamic_pointer_cast<BulletCollision>(
      this->shapeDPtr->collisionParent);

  /// Bullet requires the half-extents of the box
  btCollisionShape *shape = bParent->CollisionShape();
  if (!shape)
  {
    this->initialSize = size;
    bParent->SetCollisionShape(new btBoxShape(
          btVector3(size.X()*0.5, size.Y()*0.5, size.Z()*0.5)));
  }
  else
  {
    btVector3 boxScale = shape->getLocalScaling();
    boxScale.setX(size.X() / this->initialSize.X());
    boxScale.setY(size.Y() / this->initialSize.Y());
    boxScale.setZ(size.Z() / this->initialSize.Z());

    shape->setLocalScaling(boxScale);

    // clear bullet cache and re-add the collision shape
    // otherwise collisions won't work properly after scaling
    BulletLinkPtr bLink =
      std::dynamic_pointer_cast<BulletLink>(bParent->Link());
    bLink->ClearCollisionCache();

    // remove and add the shape again
    if (bLink->BtLink()->getCollisionShape()->isCompound())
    {
      btCompoundShape *compoundShape =
        dynamic_cast<btCompoundShape *>(bLink->BtLink()->getCollisionShape());

      compoundShape->removeChildShape(shape);
      ignition::math::Pose3d relativePose =
        this->shapeDPtr->collisionParent->RelativePose();
      relativePose.Pos() -= bLink->Inertia().CoG();
      compoundShape->addChildShape(
          BulletTypes::ConvertPose(relativePose), shape);
    }
  }
}
