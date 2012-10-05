/*
 * Copyright 2011 Nate Koenig
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
#ifndef _ODEPLANESHAPE_HH_
#define _ODEPLANESHAPE_HH_

#include "physics/PlaneShape.hh"
#include "physics/ode/ODEPhysics.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief An ODE Plane shape
    class ODEPlaneShape : public PlaneShape
    {
      /// \brief Constructor
      public: ODEPlaneShape(CollisionPtr _parent) : PlaneShape(_parent) {}

      /// \brief Destructor
      public: virtual ~ODEPlaneShape() {}

      /// \brief Create the plane
      public: void CreatePlane()
      {
        PlaneShape::CreatePlane();
        ODECollisionPtr oParent;
        oParent =
          boost::shared_dynamic_cast<ODECollision>(this->collisionParent);

        double altitude = 0;

        math::Vector3 n = this->GetNormal();
        if (oParent->GetCollisionId() == NULL)
          oParent->SetCollision(dCreatePlane(oParent->GetSpaceId(),
                n.x, n.y, n.z, altitude), false);
        else
          dGeomPlaneSetParams(oParent->GetCollisionId(),
                              n.x, n.y, n.z, altitude);
      }

      /// Set the altitude of the plane
      public: void SetAltitude(const math::Vector3 &pos)
      {
        PlaneShape::SetAltitude(pos);
        ODECollisionPtr odeParent;
        odeParent =
          boost::shared_dynamic_cast<ODECollision>(this->collisionParent);

        dVector4 vec4;

        dGeomPlaneGetParams(odeParent->GetCollisionId(), vec4);

        // Compute "altitude": scalar product of position and normal
        vec4[3] = vec4[0] * pos.x + vec4[1] * pos.y + vec4[2] * pos.z;

        dGeomPlaneSetParams(odeParent->GetCollisionId(), vec4[0], vec4[1],
                            vec4[2], vec4[3]);
      }
    };
    /// \}
  }
}
#endif






