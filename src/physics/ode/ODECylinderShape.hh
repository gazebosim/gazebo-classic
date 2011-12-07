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
#ifndef ODECYLINDERSHAPE_HH
#define ODECYLINDERSHAPE_HH

#include "physics/CylinderShape.hh"
#include "physics/ode/ODEPhysics.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{
    
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief ODE cylinder shape 
    class ODECylinderShape : public CylinderShape
    {
      public: ODECylinderShape(CollisionPtr parent) : CylinderShape(parent) {}
      public: virtual ~ODECylinderShape() {}
      public: void SetSize(const double &radius, const double &length)
      {
        CylinderShape::SetSize(radius, length);
        ODECollisionPtr oParent;
        oParent =
          boost::shared_dynamic_cast<ODECollision>(this->collisionParent);
  
        if (oParent->GetCollisionId() == NULL)
          oParent->SetCollision(dCreateCylinder( 0, radius, length ), true );
        else
          dGeomCylinderSetParams(oParent->GetCollisionId(), radius, length);
      }
    };

    /// \}
    /// \}
  }
}
#endif
