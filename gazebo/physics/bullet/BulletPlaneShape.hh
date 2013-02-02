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
/* Desc: Plane shape
 * Author: Nate Koenig
 * Date: 14 Oct 2009
 */

#ifndef _BULLETPLANESHAPE_HH_
#define _BULLETPLANESHAPE_HH_

#include <iostream>

#include "physics/bullet/BulletPhysics.hh"
#include "physics/PlaneShape.hh"

// Note that this shape is not officially supported for use with
// compound shapes: http://code.google.com/p/bullet/issues/detail?id=348
// It appears to work, but this should be kept in mind.
namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Bullet collision for an infinite plane.
    class BulletPlaneShape : public PlaneShape
    {
      /// \brief Constructor
      public: BulletPlaneShape(CollisionPtr _parent) : PlaneShape(_parent) {}

      /// \brief Destructor
      public: virtual ~BulletPlaneShape() {}

      /// \brief Set the altitude of the plane
      public: void SetAltitude(const math::Vector3 &pos)
              {
                // This function doesn't actually alter the bullet plane shape
                // The API needs to change so that the altitude is given to
                // CreatePlane()
                // Or, this child could reset the parent pointer,
                // but it currently cannot access it
                PlaneShape::SetAltitude(pos);
              }

      /// \brief Create the plane
      public: void CreatePlane()
              {
                PlaneShape::CreatePlane();
                BulletCollisionPtr bParent;
                bParent = boost::shared_dynamic_cast<BulletCollision>(
                    this->collisionParent);

                btVector3 n = BulletTypes::ConvertVector3(this->GetNormal());

                bParent->SetCollisionShape(btCollisionShapePtr(new
                  btStaticPlaneShape(n, 0.0)));
              }
    };
    /// \}
  }
}
#endif
