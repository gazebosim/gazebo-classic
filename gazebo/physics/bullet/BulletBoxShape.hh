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
/* Desc: Box shape
 * Author: Nate Koenig
 * Date: 14 Oct 2009
 */

#ifndef _BULLETBOXSHAPE_HH_
#define _BULLETBOXSHAPE_HH_

#include "physics/bullet/BulletPhysics.hh"
#include "physics/BoxShape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Bullet box collision
    class BulletBoxShape : public BoxShape
    {
      /// \brief Constructor
      public: BulletBoxShape(CollisionPtr _parent) : BoxShape(_parent) {}

      /// \brief Destructor
      public: virtual ~BulletBoxShape() {}

      /// \brief Set the size of the box
      public: void SetSize(const math::Vector3 &_size)
              {
                BulletCollisionPtr bParent;
                bParent = boost::shared_dynamic_cast<BulletCollision>(
                    this->collisionParent);
                btCollisionShapePtr shape = bParent->GetCollisionShape();

                if (shape)
                {
                  // Collision shape already exists, so resize
                  math::Vector3 oldScaling = BulletTypes::ConvertVector3(
                    shape.get()->getLocalScaling());
                  math::Vector3 oldSize = this->GetSize();
                  shape.get()->setLocalScaling(BulletTypes::ConvertVector3(
                    _size / oldSize * oldScaling));
                  // Need to call a function here to reset Bullet contacts
                  // http://code.google.com/p/bullet/issues/detail?id=687#c2
                }
                else
                {
                  // Collision shape doesn't exist, so create one
                  // Bullet requires the half-extents of the box
                  bParent->SetCollisionShape(
                    btCollisionShapePtr(new btBoxShape(
                      BulletTypes::ConvertVector3(_size*0.5))));
                }
                // Do this last so the old size is still available
                BoxShape::SetSize(_size);
              }
    };
    /// \}
  }
}
#endif
