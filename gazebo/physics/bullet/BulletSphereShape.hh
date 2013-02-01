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
/* Desc: Sphere collisionetry
 * Author: Nate Koenig
 * Date: 21 May 2009
 */

#ifndef _BULLETSPHERESHAPE_HH_
#define _BULLETSPHERESHAPE_HH_

#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/SphereShape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Bullet sphere collision
    class BulletSphereShape : public SphereShape
    {
      /// \brief Constructor, nothing here. Memory is allocated by SetRadius
      public: BulletSphereShape(CollisionPtr _parent) : SphereShape(_parent) {}

      /// \brief Destructor
      public: virtual ~BulletSphereShape() {}

      /// \brief Set the radius
      public: void SetRadius(double _radius)
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
                  double oldRadius = this->GetRadius();
                  shape.get()->setLocalScaling(BulletTypes::ConvertVector3(
                    _radius / oldRadius * oldScaling));
                  // Need to call a function here to reset Bullet contacts
                  // http://code.google.com/p/bullet/issues/detail?id=687#c2
                }
                else
                {
                  // Collision shape doesn't exist, so create one
                  bParent->SetCollisionShape(
                    btCollisionShapePtr(new btSphereShape(_radius)));
                }
                // Do this last so the old size is still available
                SphereShape::SetRadius(_radius);
              }
    };
    /// \}
  }
}
#endif
