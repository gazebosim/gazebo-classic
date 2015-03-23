/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _ODESPHERESHAPE_HH_
#define _ODESPHERESHAPE_HH_

#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/SphereShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief A ODE sphere shape
    class GZ_PHYSICS_VISIBLE ODESphereShape : public SphereShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit ODESphereShape(ODECollisionPtr _parent)
              : SphereShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~ODESphereShape() {}

      // Documentation inherited.
      public: virtual void SetRadius(double _radius)
      {
        SphereShape::SetRadius(_radius);
        ODECollisionPtr oParent;
        oParent =
          boost::dynamic_pointer_cast<ODECollision>(this->collisionParent);

        // Create the sphere geometry
        if (oParent->GetCollisionId() == NULL)
          oParent->SetCollision(dCreateSphere(0, _radius), true);
        else
          dGeomSphereSetRadius(oParent->GetCollisionId(), _radius);
      }
    };
  }
}
#endif
