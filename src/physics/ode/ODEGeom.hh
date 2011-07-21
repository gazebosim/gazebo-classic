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
/* Desc: Geom class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#ifndef ODEGEOM_HH
#define ODEGEOM_HH

#include "physics/ode/ode_inc.h"

#include "common/CommonTypes.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/Geom.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{
    
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief Base class for all ODE geoms
    class ODEGeom : public Geom
    {
      /// \brief Constructor
      public: ODEGeom(BodyPtr body);
    
      /// \brief Destructor
      public: virtual ~ODEGeom();
  
      /// \brief Load the geom
      public: virtual void Load( sdf::ElementPtr &_sdf );
  
      /// \brief Set the encapsulated geometry object
      public: void SetGeom(dGeomID geomId, bool placeable);
    
      /// \brief Return the geom id
      /// \return The geom id
      public: dGeomID GetGeomId() const;
    
      /// \brief Get the ODE geom class
      public: int GetGeomClass() const;
    
      public: virtual void OnPoseChange();
  
      /// \brief Set the category bits, used during collision detection
      /// \param bits The bits
      public: virtual void SetCategoryBits(unsigned int bits);
    
      /// \brief Set the collide bits, used during collision detection
      /// \param bits The bits
      public: virtual void SetCollideBits(unsigned int bits);
    
      /// \brief Get the bounding box, defined by the physics engine
      public: virtual math::Box GetBoundingBox() const;
  
      /// \brief Get the geom's space ID
      public: dSpaceID GetSpaceId() const;
  
      /// \brief Set the geom's space ID
      public: void SetSpaceId(dSpaceID spaceid);
  
      protected: dSpaceID spaceId;
  
      ///  ID for the sub-geom
      protected: dGeomID geomId;
    };
  
    /// \}
    /// \}
  
  }
}
#endif
