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
#ifndef _ODESPHERESHAPE_HH_
#define _ODESPHERESHAPE_HH_

#include "physics/ode/ODEPhysics.hh"
#include "physics/ode/ODECollision.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/SphereShape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief A ODE sphere shape
    class ODESphereShape : public SphereShape
    {
      /// \brief Constructor
      public: ODESphereShape(ODECollisionPtr _parent) : SphereShape(_parent) {}

      /// \brief Destructor
      public: virtual ~ODESphereShape() {}

      /// \brief Set the radius
      public: void SetRadius(const double &_radius)
      {
        SphereShape::SetRadius(_radius);
        ODECollisionPtr oParent;
        oParent =
          boost::shared_dynamic_cast<ODECollision>(this->collisionParent);

        // Create the sphere geometry
        if (oParent->GetCollisionId() == NULL)
          oParent->SetCollision(dCreateSphere(0, _radius), true);
        else
          dGeomSphereSetRadius(oParent->GetCollisionId(), _radius);
      }
    };
    /// \}
  }
}
#endif
