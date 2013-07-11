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

#ifndef _SIMBODY_CYLINDERSHAPE_HH_
#define _SIMBODY_CYLINDERSHAPE_HH_

#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/CylinderShape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Cylinder collision
    class SimbodyCylinderShape : public CylinderShape
    {
      /// \brief Constructor
      public: SimbodyCylinderShape(CollisionPtr _parent)
              : CylinderShape(_parent) {}

      /// \brief Destructor
      public: virtual ~SimbodyCylinderShape() {}

      /// \brief Set the size of the cylinder
      public: void SetSize(double _radius, double _length)
              {
                CylinderShape::SetSize(_radius, _length);
                SimbodyCollisionPtr bParent;
                bParent = boost::shared_dynamic_cast<SimbodyCollision>(
                    this->collisionParent);

                // set collision shape
              }
    };
    /// \}
  }
}
#endif
