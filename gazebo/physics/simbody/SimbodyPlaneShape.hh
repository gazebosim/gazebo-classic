/*
 * Copyright 2013 Open Source Robotics Foundation
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

#ifndef _SIMBODY_PLANESHAPE_HH_
#define _SIMBODY_PLANESHAPE_HH_

#include <iostream>

#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/PlaneShape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Simbody collision for an infinite plane.
    class SimbodyPlaneShape : public PlaneShape
    {
      /// \brief Constructor
      public: SimbodyPlaneShape(CollisionPtr _parent) : PlaneShape(_parent) {}

      /// \brief Destructor
      public: virtual ~SimbodyPlaneShape() {}

      /// \brief Set the altitude of the plane
      public: void SetAltitude(const math::Vector3 &pos)
              {
                PlaneShape::SetAltitude(pos);
              }

      /// \brief Create the plane
      public: void CreatePlane()
              {
                PlaneShape::CreatePlane();
                SimbodyCollisionPtr bParent;
                bParent = boost::dynamic_pointer_cast<SimbodyCollision>(
                    this->collisionParent);

                math::Vector3 n = this->GetNormal();

                // set collision shape
              }
    };
    /// \}
  }
}
#endif
