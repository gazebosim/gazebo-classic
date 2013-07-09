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

#ifndef _SIMBODY_BOXSHAPE_HH_
#define _SIMBODY_BOXSHAPE_HH_

#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/BoxShape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Simbody box collision
    class SimbodyBoxShape : public BoxShape
    {
      /// \brief Constructor
      public: SimbodyBoxShape(CollisionPtr _parent) : BoxShape(_parent) {}

      /// \brief Destructor
      public: virtual ~SimbodyBoxShape() {}

      /// \brief Set the size of the box
      public: void SetSize(const math::Vector3 &_size)
              {
                BoxShape::SetSize(_size);
                SimbodyCollisionPtr bParent;
                bParent = boost::shared_dynamic_cast<SimbodyCollision>(
                    this->collisionParent);

                /// Simbody requires the half-extents of the box
              }
    };
    /// \}
  }
}
#endif
