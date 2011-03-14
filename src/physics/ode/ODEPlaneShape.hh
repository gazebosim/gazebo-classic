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
#ifndef ODEPLANESHAPE_HH
#define ODEPLANESHAPE_HH

#include "Body.hh"
#include "Mass.hh"
#include "ODEPhysics.hh"
#include "PlaneShape.hh"

namespace gazebo
{
	namespace physics
  {
    class ODEPlaneShape : public PlaneShape
    {
      public: ODEPlaneShape(Geom *parent) : PlaneShape(parent) {}
      public: virtual ~ODEPlaneShape() {}
    
      /// \brief Create the plane
      public: void CreatePlane()
      {
        PlaneShape::CreatePlane();
        ODEGeom *odeParent = (ODEGeom*)(this->geomParent);
    
        double altitude = 0;
  
        odeParent->SetGeom(dCreatePlane(odeParent->GetSpaceId(), 
                           (**normalP).x, (**normalP).y, (**normalP).z, 
                           altitude), false);
      }
    
      /// Set the altitude of the plane
      public: void SetAltitude(const Vector3 &pos)
      {
        PlaneShape::SetAltitude(pos);
        ODEGeom *odeParent = (ODEGeom*)(this->geomParent);
  
        dVector4 vec4;
    
        dGeomPlaneGetParams(odeParent->GetGeomId(), vec4);
    
        // Compute "altitude": scalar product of position and normal
        vec4[3] = vec4[0] * pos.x + vec4[1] * pos.y + vec4[2] * pos.z;
        dGeomPlaneSetParams(odeParent->GetGeomId(), vec4[0], vec4[1], 
                            vec4[2], vec4[3]);
      }
    };
  }
}
#endif
