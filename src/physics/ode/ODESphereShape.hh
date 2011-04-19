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
#include "physics/ode/ODEGeom.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/SphereShape.hh"

namespace gazebo
{
	namespace physics
  {
    class ODESphereShape : public SphereShape
    {
      public: ODESphereShape(ODEGeomPtr parent) : SphereShape(parent) {}
      public: virtual ~ODESphereShape() {}
      public: void SetSize(const double &radius)
      {
        SphereShape::SetSize(radius);
        ODEGeomPtr oParent;
        oParent = boost::shared_dynamic_cast<ODEGeom>(this->geomParent);
  
        dMass odeMass;
        common::Pose3d rpose;
    
        Mass mass = this->geomParent->GetMass();

        // Initialize box mass matrix
        dMassSetSphereTotal(&odeMass, mass.GetAsDouble(), radius);
        rpose = this->geomParent->GetRelativePose();
        dMassTranslate(&odeMass, rpose.pos.x, rpose.pos.y, rpose.pos.z);
    
        ODEPhysics::ConvertMass(&mass, &odeMass);
        
        this->geomParent->SetMass(mass);
    
        // Create the sphere geometry
        oParent->SetGeom( dCreateSphere(0, radius), true);
      }
    
    };
  }
}
#endif
