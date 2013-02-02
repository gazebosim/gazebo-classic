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
      /// \brief Constructor
      public: BulletCylinderShape(CollisionPtr _parent)
              : CylinderShape(_parent) {}

      /// \brief Destructor
      public: virtual ~BulletCylinderShape() {}

      /// \brief Set the size of the cylinder
      public: void SetSize(double _radius, double _length)
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
                  double radiusRatio = _radius / this->GetRadius();
                  double lengthRatio = _length / this->GetLength();
                  math::Vector3 ratio(_radius / oldRadius, _radius / 
                  shape.get()->setLocalScaling(BulletTypes::ConvertVector3(
                    math::Vector3(radiusRatio, radiusRatio, lengthRatio) *
                      oldScaling));
                  // Need to call a function here to reset Bullet contacts
                  // http://code.google.com/p/bullet/issues/detail?id=687#c2
                }
                else
                {
                  // Collision shape doesn't exist, so create one
                  // Bullet requires the half-extents of the shape
                  bParent->SetCollisionShape(
                    btCollisionShapePtr(new btCylinderShape(
                      btVector3(_radius, _radius, _length * 0.5)));
                }
                // Do this last so the old size is still available
                CylinderShape::SetSize(_radius, _length);
              }
    };
    /// \}
  }
}
#endif
