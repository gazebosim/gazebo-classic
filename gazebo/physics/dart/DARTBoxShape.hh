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
#ifndef _DARTBOXSHAPE_HH_
#define _DARTBOXSHAPE_HH_

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
      public: virtual void SetSize(const math::Vector3 &/*_size*/)
      {
//         BoxShape::SetSize(_size);
// 
//         DARTCollisionPtr oParent;
//         oParent = boost::shared_dynamic_cast<DARTCollision>(
//             this->collisionParent);
// 
//         if (oParent->GetCollisionId() == NULL)
//           oParent->SetCollision(dCreateBox(0, _size.x, _size.y, _size.z), true);
//         else
//           dGeomBoxSetLengths(oParent->GetCollisionId(),
//                              _size.x, _size.y, _size.z);
      }
    };
  }
}
#endif
