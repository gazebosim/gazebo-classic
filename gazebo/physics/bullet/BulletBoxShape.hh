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
      /// \brief Constructor, empty because the box is created by SetSize.
      public: BulletBoxShape(CollisionPtr _parent) : BoxShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~BulletBoxShape() {
                BulletCollisionPtr bParent;
                bParent = boost::shared_dynamic_cast<BulletCollision>(
                    this->collisionParent);
                if (bParent && bParent->GetCollisionShape()) {
                  delete bParent->GetCollisionShape();
                  bParent->SetCollisionShape(NULL);
                }
              }

      /// \brief Set the size of the box. This is also where the box is created
      ///        when SetSize is run for the first time.
      public: void SetSize(const math::Vector3 &_size)
              {
                BulletCollisionPtr bParent;
                bParent = boost::shared_dynamic_cast<BulletCollision>(
                    this->collisionParent);
                btCollisionShape *bShape = bParent->GetCollisionShape();

                if (!bShape) {
                  // Create the box if it doesn't yet exist.
                  // Bullet requires the half-extents of the box
                  bParent->SetCollisionShape(new btBoxShape(
                      btVector3(_size.x*0.5, _size.y*0.5, _size.z*0.5)));
                } else {
                  // Re-scale the existing btBoxShape
                  math::Vector3 scaling = _size / this->GetSize();
                  btVector3 old_scaling = bShape->getLocalScaling();
                  bShape->setLocalScaling(btVector3(
                    btScalar(scaling.x) * old_scaling.getX(),
                    btScalar(scaling.y) * old_scaling.getY(),
                    btScalar(scaling.z) * old_scaling.getZ()));
                }
                // Do this last so the old size is available above, if necessary
                BoxShape::SetSize(_size);
              }
    };
    /// \}
  }
}
#endif
