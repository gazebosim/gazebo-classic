/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef _SIMBODY_CYLINDERSHAPE_HH_
#define _SIMBODY_CYLINDERSHAPE_HH_

#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/CylinderShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Cylinder collision
    class GZ_PHYSICS_VISIBLE SimbodyCylinderShape : public CylinderShape
    {
      /// \brief Constructor
      public: SimbodyCylinderShape(CollisionPtr _parent)
              : CylinderShape(_parent) {}

      /// \brief Destructor
      public: virtual ~SimbodyCylinderShape() {}

      // Documentation inherited
      public: void SetSize(double _radius, double _length)
              {
                if (_radius < 0)
                {
                  gzerr << "Cylinder shape does not support negative radius\n";
                  return;
                }
                if (_length < 0)
                {
                  gzerr << "Cylinder shape does not support negative length\n";
                  return;
                }
                if (math::equal(_radius, 0.0))
                {
                  // Warn user, but still create shape with very small value
                  // otherwise later resize operations using setLocalScaling
                  // will not be possible
                  gzwarn << "Setting cylinder shape's radius to zero \n";
                  _radius = 1e-4;
                }
                if (math::equal(_length, 0.0))
                {
                  gzwarn << "Setting cylinder shape's length to zero \n";
                  _length = 1e-4;
                }

                CylinderShape::SetSize(_radius, _length);
                SimbodyCollisionPtr bParent;
                bParent = boost::dynamic_pointer_cast<SimbodyCollision>(
                    this->collisionParent);

                // set collision shape
              }
    };
    /// \}
  }
}
#endif
