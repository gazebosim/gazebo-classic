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
#ifndef ODEMULTIRAYSHAPE_HH
#define ODEMULTIRAYSHAPE_HH

#include "physics/MultiRayShape.hh"

namespace gazebo
{
	namespace physics
  {
    /// \brief ODE specific version of MultiRayShape
    class ODEMultiRayShape : public MultiRayShape
    {
      /// \brief Constructor
      public: ODEMultiRayShape(Geom *parent);
    
      /// \brief Destructor
      public: virtual ~ODEMultiRayShape();
   
      /// \brief Update the rays 
      public: virtual void UpdateRays();
  
      /// \brief Ray-intersection callback
      private: static void UpdateCallback( void *data, dGeomID o1, dGeomID o2 );
  
      /// \brief Add a ray to the geom
      protected: void AddRay(const common::Vector3 &start, const common::Vector3 &end );
  
      /// Ray space for collision detector
      private: dSpaceID superSpaceId;
      private: dSpaceID raySpaceId; 
    };
  }
}
#endif
