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

#include "Mass.hh"
#include "Vector2.hh"
#include "ODEPhysics.hh"
#include "CylinderShape.hh"

namespace gazebo
{
	namespace physics
  {
    class ODECylinderShape : public CylinderShape
    {
      public: ODECylinderShape(Geom *parent) : CylinderShape(parent) {}
      public: virtual ~ODECylinderShape() {}
      public: void SetSize(const double &radius, const double &length)
      {
        CylinderShape::SetSize(radius, length);
        PhysicsEngine *physics = this->GetWorld()->GetPhysicsEngine();
        ODEGeom *oParent = (ODEGeom*)(this->geomParent);
  
        dMass odeMass;
        Pose3d rpose;
      
        Mass mass = this->geomParent->GetMass();
    
        // Initialize mass matrix
        dMassSetCylinderTotal(&odeMass, mass.GetAsDouble(), 3, 
                              radius, length);
        rpose = this->geomParent->GetRelativePose();
        dMassTranslate(&odeMass, rpose.pos.x, rpose.pos.y, rpose.pos.z);
   
        physics->ConvertMass(&mass, &odeMass);
    
        this->geomParent->SetMass(mass);
  
        oParent->SetGeom( dCreateCylinder( 0, radius, length ), true );
      }
    };
  }
}
#endif
