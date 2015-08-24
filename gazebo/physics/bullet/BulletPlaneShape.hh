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
/* Desc: Plane shape
 * Author: Nate Koenig
 * Date: 14 Oct 2009
 */

#ifndef _BULLETPLANESHAPE_HH_
#define _BULLETPLANESHAPE_HH_

#include <iostream>

#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/PlaneShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Bullet collision for an infinite plane.
    class GZ_PHYSICS_VISIBLE BulletPlaneShape : public PlaneShape
    {
      /// \brief Constructor
      public: BulletPlaneShape(CollisionPtr _parent) : PlaneShape(_parent) {}

      /// \brief Destructor
      public: virtual ~BulletPlaneShape() {}

      /// \brief Set the altitude of the plane
      public: void SetAltitude(const math::Vector3 &pos)
              {
                PlaneShape::SetAltitude(pos);
              }

      /// \brief Create the plane
      public: void CreatePlane()
              {
                PlaneShape::CreatePlane();
                BulletCollisionPtr bParent;
                bParent = boost::dynamic_pointer_cast<BulletCollision>(
                    this->collisionParent);

                math::Vector3 n = this->GetNormal();
                btVector3 vec(n.x, n.y, n.z);

                bParent->SetCollisionShape(new btStaticPlaneShape(vec, 0.0),
                    false);
              }
    };
    /// \}
  }
}
#endif
