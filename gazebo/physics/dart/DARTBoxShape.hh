/*
 * Copyright 2014 Open Source Robotics Foundation
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
#ifndef _GAZEBO_DARTBOXSHAPE_HH_
#define _GAZEBO_DARTBOXSHAPE_HH_

#include "gazebo/common/Console.hh"

#include "gazebo/math/Vector3.hh"

#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/physics/dart/DARTCollision.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/BoxShape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief DART Box shape
    class DARTBoxShape : public BoxShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit DARTBoxShape(DARTCollisionPtr _parent)
              : BoxShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~DARTBoxShape() {}

      // Documentation inherited.
      public: virtual void SetSize(const math::Vector3 &_size)
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
          gzwarn << "Setting box shape's x to zero is not supported in DART, "
                 << "using 1e-4.\n";
          size.x = 1e-4;
        }

        if (math::equal(size.y, 0.0))
        {
          gzwarn << "Setting box shape's y to zero is not supported in DART, "
                 << "using 1e-4.\n";
          size.y = 1e-4;
        }

        if (math::equal(size.z, 0.0))
        {
          gzwarn << "Setting box shape's z to zero is not supported in DART "
                 << "using 1e-4.\n";
          size.z = 1e-4;
        }

        BoxShape::SetSize(size);

        DARTCollisionPtr dartCollisionParent =
            boost::dynamic_pointer_cast<DARTCollision>(this->collisionParent);

        if (dartCollisionParent->GetDARTCollisionShape() == NULL)
        {
          dart::dynamics::BodyNode *dtBodyNode =
              dartCollisionParent->GetDARTBodyNode();
          dart::dynamics::BoxShape *dtBoxShape =
              new dart::dynamics::BoxShape(DARTTypes::ConvVec3(size));
          dtBodyNode->addCollisionShape(dtBoxShape);
          dartCollisionParent->SetDARTCollisionShape(dtBoxShape);
        }
        else
        {
          dart::dynamics::BoxShape *dtBoxShape =
              dynamic_cast<dart::dynamics::BoxShape*>(
                dartCollisionParent->GetDARTCollisionShape());
          dtBoxShape->setDim(DARTTypes::ConvVec3(size));
        }
      }
    };
  }
}
#endif
