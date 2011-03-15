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

#include "common/Vector3.hh"

#include "physics/ode/ODEPhysics.hh"
#include "physics/Mass.hh"
#include "physics/BoxShape.hh"

namespace gazebo
{
	namespace physics
  {
    class ODEBoxShape : public BoxShape
    {
      public: ODEBoxShape(Geom *parent) : BoxShape(parent) {}
      public: virtual ~ODEBoxShape() {}
      public: virtual void SetSize( const common::Vector3 &size )
      {
        BoxShape::SetSize(size);
        ODEGeom *oParent = (ODEGeom*)(this->geomParent);
        common::Pose3d rpose;
  
        dMass odeMass;
  
        Mass mass = this->geomParent->GetMass();
    
        // Initialize box mass matrix
        dMassSetBoxTotal(&odeMass, mass.GetAsDouble(), 
                         size.x, size.y, size.z);
        rpose = this->geomParent->GetRelativePose();
        dMassTranslate(&odeMass, rpose.pos.x, rpose.pos.y, rpose.pos.z);
   
        ODEPhysics::ConvertMass(&mass, &odeMass);
  
        this->geomParent->SetMass(mass);
    
        // Create a box geometry with box mass matrix
        oParent->SetGeom( dCreateBox( 0, size.x, size.y, size.z), true );
      }
    
    
    };
  }
}
#endif
