/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _SIMBODY_BOXSHAPE_HH_
#define _SIMBODY_BOXSHAPE_HH_

#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/BoxShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Simbody box collision
    class GAZEBO_VISIBLE SimbodyBoxShape : public BoxShape
    {
      /// \brief Constructor
      public: SimbodyBoxShape(CollisionPtr _parent) : BoxShape(_parent) {}

      /// \brief Destructor
      public: virtual ~SimbodyBoxShape() {}

      // Documentation inherited
      public: void SetSize(const math::Vector3 &_size)
              {
                if (_size.x < 0 || _size.y < 0 || _size.z < 0)
                {
                  gzerr << "Box shape does not support negative size\n";
                  return;
                }
                math::Vector3 size = _size;
                if (math::equal(size.x, 0.0))
                {
                  // Warn user, but still create shape with very small value
                  // otherwise later resize operations using setLocalScaling
                  // will not be possible
                  gzwarn << "Setting box shape's x to zero \n";
                  size.x = 1e-4;
                }
                if (math::equal(size.y, 0.0))
                {
                  gzwarn << "Setting box shape's y to zero \n";
                  size.y = 1e-4;
                }
                if (math::equal(size.z, 0.0))
                {
                  gzwarn << "Setting box shape's z to zero \n";
                  size.z = 1e-4;
                }

                BoxShape::SetSize(size);

                SimbodyCollisionPtr bParent;
                bParent = boost::dynamic_pointer_cast<SimbodyCollision>(
                    this->collisionParent);
              }
    };
    /// \}
  }
}
#endif
