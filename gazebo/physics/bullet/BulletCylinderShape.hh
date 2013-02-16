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
      /// \param[in] _radius Cylinder radius
      /// \param[in] _length Cylinder length
      public: void SetSize(double _radius, double _length)
              {
                if (_radius < 0)
                {
                    gzerr << "Cylinder shape does not support negative"
                          << " radius\n";
                    return;
                }
                if (_length < 0)
                {
                    gzerr << "Cylinder shape does not support negative"
                          << " length\n";
                    return;
                }

                CylinderShape::SetSize(_radius, _length);
                BulletCollisionPtr bParent;
                bParent = boost::shared_dynamic_cast<BulletCollision>(
                    this->collisionParent);

                btCollisionShape *shape = bParent->GetCollisionShape();
                if (!shape)
                {
                  bParent->SetCollisionShape(new btCylinderShapeZ(
                      btVector3(_radius, _radius, _length * 0.5)));
                }
                else
                {
                  btVector3 scale = shape->getLocalScaling();
                  double cylinderRadius = this->GetRadius();
                  double cylinderLength = this->GetLength();
                  if (cylinderRadius > 0)
                  {
                    scale.setX(_radius / cylinderRadius);
                    scale.setY(scale.x());
                  }
                  if (cylinderLength > 0)
                    scale.setZ(_length / cylinderLength);
                  shape->setLocalScaling(scale);
                }
              }
    };
    /// \}
  }
}
#endif
