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
#ifndef _RTQL8PLANESHAPE_HH_
#define _RTQL8PLANESHAPE_HH_

#include "gazebo/physics/PlaneShape.hh"
#include "gazebo/physics/rtql8/RTQL8Physics.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief An RTQL8 Plane shape.
    class RTQL8PlaneShape : public PlaneShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit RTQL8PlaneShape(CollisionPtr _parent)
              : PlaneShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~RTQL8PlaneShape() {}

      // Documentation inherited
      public: virtual void CreatePlane()
      {
//         PlaneShape::CreatePlane();
//         RTQL8CollisionPtr oParent;
//         oParent =
//           boost::shared_dynamic_cast<RTQL8Collision>(this->collisionParent);
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
      public: virtual void SetAltitude(const math::Vector3 &_pos)
      {
//         PlaneShape::SetAltitude(_pos);
//         RTQL8CollisionPtr odeParent;
//         odeParent =
//           boost::shared_dynamic_cast<RTQL8Collision>(this->collisionParent);
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
