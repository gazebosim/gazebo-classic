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
#ifndef _DARTPLANESHAPE_HH_
#define _DARTPLANESHAPE_HH_

#include "gazebo/physics/PlaneShape.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief An DART Plane shape.
    class DARTPlaneShape : public PlaneShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit DARTPlaneShape(CollisionPtr _parent)
              : PlaneShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~DARTPlaneShape() {}

      // Documentation inherited
      public: virtual void CreatePlane()
      {
//         PlaneShape::CreatePlane();
//         DARTCollisionPtr oParent;
//         oParent =
//           boost::shared_dynamic_cast<DARTCollision>(this->collisionParent);
// 
//         double altitude = 0;
// 
//         math::Vector3 n = this->GetNormal();
//         if (oParent->GetCollisionId() == NULL)
//           oParent->SetCollision(dCreatePlane(oParent->GetSpaceId(),
//                 n.x, n.y, n.z, altitude), false);
//         else
//           dGeomPlaneSetParams(oParent->GetCollisionId(),
//                               n.x, n.y, n.z, altitude);
      }

      // Documentation inherited
      public: virtual void SetAltitude(const math::Vector3 &/*_pos*/)
      {
//         PlaneShape::SetAltitude(_pos);
//         DARTCollisionPtr odeParent;
//         odeParent =
//           boost::shared_dynamic_cast<DARTCollision>(this->collisionParent);
// 
//         dVector4 vec4;
// 
//         dGeomPlaneGetParams(odeParent->GetCollisionId(), vec4);
// 
//         // Compute "altitude": scalar product of position and normal
//         vec4[3] = vec4[0] * _pos.x + vec4[1] * _pos.y + vec4[2] * _pos.z;
// 
//         dGeomPlaneSetParams(odeParent->GetCollisionId(), vec4[0], vec4[1],
//                             vec4[2], vec4[3]);
      }
    };
  }
}
#endif
