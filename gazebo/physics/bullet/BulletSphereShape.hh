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

#include "physics/bullet/BulletPhysics.hh"
#include "physics/SphereShape.hh"

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
      /// \brief Constructor, empty because the shape is created by SetSize.
      public: BulletSphereShape(CollisionPtr _parent) : SphereShape(_parent) {}

      /// \brief Destructor
      public: virtual ~BulletSphereShape()
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

      /// \brief Set the radius of the sphere. This is also where the shape is
      ///        created when SetRadius is called for the first time.
      /// \param[in] _radius Radius of the sphere.
      public: void SetRadius(double _radius)
              {
                SphereShape::SetRadius(_radius);
                BulletCollisionPtr bParent;
                bParent = boost::shared_dynamic_cast<BulletCollision>(
                    this->collisionParent);
                btCollisionShape *bShape = bParent->GetCollisionShape();

                if (!bShape)
                {
                  // Create the shape if it doesn't yet exist
                  bParent->SetCollisionShape(new btSphereShape(
                    btScalar(_radius)));
                }
                else
                {
                  // Re-size the existing btSphereShape
                  static_cast<btSphereShape*>(bShape)->setUnscaledRadius(
                    btScalar(_radius));
                }
              }
    };
    /// \}
  }
}
#endif
