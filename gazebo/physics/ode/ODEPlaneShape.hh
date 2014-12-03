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
#ifndef _ODEPLANESHAPE_HH_
#define _ODEPLANESHAPE_HH_

#include "gazebo/physics/PlaneShape.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief An ODE Plane shape.
    class GZ_PHYSICS_ODE_VISIBLE ODEPlaneShape : public PlaneShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit ODEPlaneShape(CollisionPtr _parent)
              : PlaneShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~ODEPlaneShape() {}

      // Documentation inherited
      public: virtual void CreatePlane()
      {
        PlaneShape::CreatePlane();
        ODECollisionPtr oParent;
        oParent =
          boost::dynamic_pointer_cast<ODECollision>(this->collisionParent);
        math::Pose pose = oParent->GetWorldPose();
        double altitude = pose.pos.z;
        math::Vector3 n = this->GetNormal();
        if (oParent->GetCollisionId() == NULL)
          oParent->SetCollision(dCreatePlane(oParent->GetSpaceId(),
                n.x, n.y, n.z, altitude), false);
        else
          dGeomPlaneSetParams(oParent->GetCollisionId(),
                              n.x, n.y, n.z, altitude);
      }

      // Documentation inherited
      public: virtual void SetAltitude(const math::Vector3 &_pos)
      {
        PlaneShape::SetAltitude(_pos);
        ODECollisionPtr odeParent;
        odeParent =
          boost::dynamic_pointer_cast<ODECollision>(this->collisionParent);

        dVector4 vec4;

        dGeomPlaneGetParams(odeParent->GetCollisionId(), vec4);

        // Compute "altitude": scalar product of position and normal
        vec4[3] = vec4[0] * _pos.x + vec4[1] * _pos.y + vec4[2] * _pos.z;

        dGeomPlaneSetParams(odeParent->GetCollisionId(), vec4[0], vec4[1],
                            vec4[2], vec4[3]);
      }
    };
  }
}
#endif
