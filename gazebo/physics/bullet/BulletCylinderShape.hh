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
/* Desc: Cylinder shape
 * Author: Nate Koenig
 * Date: 14 Oct 2009
 */

#ifndef _BULLETCYLINDERSHAPE_HH_
#define _BULLETCYLINDERSHAPE_HH_

#include "physics/bullet/BulletPhysics.hh"
#include "physics/CylinderShape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Cylinder collision
    class BulletCylinderShape : public CylinderShape
    {
      /// \brief Constructor, empty because the shape is created by SetSize.
      public: BulletCylinderShape(CollisionPtr _parent)
              : CylinderShape(_parent) {}

      /// \brief Destructor
      public: virtual ~BulletCylinderShape()
              {
                BulletCollisionPtr bParent;
                bParent = boost::shared_dynamic_cast<BulletCollision>(
                    this->collisionParent);
                if (bParent && bParent->GetCollisionShape())
                {
                  delete bParent->GetCollisionShape();
                  bParent->SetCollisionShape(NULL);
                }
              }

      /// \brief Set the size of the shape. This is also where the shape is
      ///        created when SetSize is run for the first time.
      /// \param[in] _radius New radius.
      /// \param[in] _length New length.
      public: void SetSize(double _radius, double _length)
              {
                BulletCollisionPtr bParent;
                bParent = boost::shared_dynamic_cast<BulletCollision>(
                    this->collisionParent);
                btCollisionShape *bShape = bParent->GetCollisionShape();

                if (!bShape)
                {
                  // Create the shape if it doesn't yet exist.
                  // Bullet requires the half-extents of the shape.
                  bParent->SetCollisionShape(new btCylinderShapeZ(
                      btVector3(btScalar(_radius), btScalar(_radius),
                        btScalar(_length * 0.5))));
                }
                else
                {
                  // Re-scale the existing btCylinderShape
                  math::Vector3 scaling;
                  scaling.x = _radius / this->GetRadius();
                  scaling.y = scaling.x;
                  scaling.z = _length / this->GetLength();
                  btVector3 old_scaling = bShape->getLocalScaling();
                  bShape->setLocalScaling(btVector3(
                    btScalar(scaling.x) * old_scaling.getX(),
                    btScalar(scaling.y) * old_scaling.getY(),
                    btScalar(scaling.z) * old_scaling.getZ()));
                }

                // Do this last so the old size is available above, if necessary
                CylinderShape::SetSize(_radius, _length);
              }
    };
    /// \}
  }
}
#endif
