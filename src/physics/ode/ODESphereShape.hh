/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef ODESPHERESHAPE_HH
#define ODESPHERESHAPE_HH

#include "physics/ode/ODEPhysics.hh"
#include "physics/ode/ODECollision.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/SphereShape.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{
    
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief And ODE sphere shape
    class ODESphereShape : public SphereShape
    {
      public: ODESphereShape(ODECollisionPtr parent) : SphereShape(parent) {}
      public: virtual ~ODESphereShape() {}
      public: void SetSize(const double &radius)
      {
        SphereShape::SetSize(radius);
        ODECollisionPtr oParent;
        oParent = boost::shared_dynamic_cast<ODECollision>(this->collisionParent);
  
        // Create the sphere geometry
        oParent->SetCollision( dCreateSphere(0, radius), true);
      }
    
    };
    /// \}
    /// \}

  }
}
#endif
