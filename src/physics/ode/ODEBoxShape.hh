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
#ifndef ODEBOXSHAPE_HH
#define ODEBOXSHAPE_HH

#include "math/Vector3.hh"

#include "physics/ode/ODEPhysics.hh"
#include "physics/ode/ODETypes.hh"
#include "physics/ode/ODEGeom.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/BoxShape.hh"

namespace gazebo
{
	namespace physics
  {
    class ODEBoxShape : public BoxShape
    {
      public: ODEBoxShape(ODEGeomPtr parent) : BoxShape(parent) {}
      public: virtual ~ODEBoxShape() {}
      public: virtual void SetSize( const math::Vector3 &size )
      {
        BoxShape::SetSize(size);

        ODEGeomPtr oParent;
        oParent = boost::shared_dynamic_cast<ODEGeom>(this->geomParent);

        oParent->SetGeom( dCreateBox( 0, size.x, size.y, size.z), true );
      }
    
    
    };
  }
}
#endif
