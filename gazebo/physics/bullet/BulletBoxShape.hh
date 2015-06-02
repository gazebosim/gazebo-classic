/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
/* Desc: Box shape
 * Author: Nate Koenig
 * Date: 14 Oct 2009
 */

#ifndef _BULLETBOXSHAPE_HH_
#define _BULLETBOXSHAPE_HH_

#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/BoxShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Bullet box collision
    class GZ_PHYSICS_VISIBLE BulletBoxShape : public BoxShape
    {
      /// \brief Constructor
      public: BulletBoxShape(CollisionPtr _parent) : BoxShape(_parent) {}

      /// \brief Destructor
      public: virtual ~BulletBoxShape() {}

      /// \brief Set the size of the box
      public: void SetSize(const math::Vector3 &_size)
              {
                if (_size.x < 0 || _size.y < 0 || _size.z < 0)
                {
                  gzerr << "Box shape does not support negative size\n";
                  return;
                }
                math::Vector3 size = _size;
                if (math::equal(size.x, 0.0))
                {
                  // Warn user, but still create shape with very small value
                  // otherwise later resize operations using setLocalScaling
                  // will not be possible
                  gzwarn << "Setting box shape's x to zero \n";
                  size.x = 1e-4;
                }
                if (math::equal(size.y, 0.0))
                {
                  gzwarn << "Setting box shape's y to zero \n";
                  size.y = 1e-4;
                }
                if (math::equal(size.z, 0.0))
                {
                  gzwarn << "Setting box shape's z to zero \n";
                  size.z = 1e-4;
                }

                BoxShape::SetSize(size);
                BulletCollisionPtr bParent;
                bParent = boost::dynamic_pointer_cast<BulletCollision>(
                    this->collisionParent);

                /// Bullet requires the half-extents of the box
                btCollisionShape *shape = bParent->GetCollisionShape();
                if (!shape)
                {
                  this->initialSize = size;
                  bParent->SetCollisionShape(new btBoxShape(
                      btVector3(size.x*0.5, size.y*0.5, size.z*0.5)));
                }
                else
                {
                  btVector3 boxScale = shape->getLocalScaling();
                  boxScale.setX(size.x / this->initialSize.x);
                  boxScale.setY(size.y / this->initialSize.y);
                  boxScale.setZ(size.z / this->initialSize.z);

                  shape->setLocalScaling(boxScale);

                  // clear bullet cache and re-add the collision shape
                  // otherwise collisions won't work properly after scaling
                  BulletLinkPtr bLink =
                    boost::dynamic_pointer_cast<BulletLink>(
                    bParent->GetLink());
                  bLink->ClearCollisionCache();

                  // remove and add the shape again
                  if (bLink->GetBulletLink()->getCollisionShape()->isCompound())
                  {
                    btCompoundShape *compoundShape =
                        dynamic_cast<btCompoundShape *>(
                        bLink->GetBulletLink()->getCollisionShape());

                    compoundShape->removeChildShape(shape);
                    math::Pose relativePose =
                        this->collisionParent->GetRelativePose();
                    relativePose.pos -= bLink->GetInertial()->GetCoG();
                    compoundShape->addChildShape(
                        BulletTypes::ConvertPose(relativePose), shape);
                  }
                }
              }

      /// \brief Initial size of box.
      private: math::Vector3 initialSize;
    };
    /// \}
  }
}
#endif
