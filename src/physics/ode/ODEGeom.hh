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
 * SVN: $Id: Geom.hh 7640 2009-05-13 02:06:08Z natepak $
 */

#ifndef ODEGEOM_HH
#define ODEGEOM_HH

#include "ODEPhysics.hh"
#include "common/Param.hh"
#include "Entity.hh"
#include "common/Pose3d.hh"
#include "common/Vector3.hh"
#include "Geom.hh"

namespace gazebo
{
	namespace physics
  {
  
    class Model;
    class Body;
    class ContactParams;
    class XMLConfigNode;
    class Visual;
    class PhysicsEngine;
  
    /// \addtogroup gazebo_physics_ode
    /// \brief Base class for all ODE geoms
    /// \{
  
    /// \brief Base class for all geoms
    class ODEGeom : public Geom
    {
    
      /// \brief Constructor
      //public: Geom(Body *body, const std::string &name);
      public: ODEGeom(Body *body);
    
      /// \brief Destructor
      public: virtual ~ODEGeom();
  
      /// \brief Load the geom
      public: virtual void Load(XMLConfigNode *node);
  
      /// \brief Load the geom
      public: virtual void Save(std::string &prefix, std::ostream &stream);
  
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
    
      /// \brief Get the mass of the geom
      public: Mass GetBodyMassMatrix();
    
      /// \brief Get the bounding box, defined by the physics engine
      public: virtual void GetBoundingBox( Vector3 &min, Vector3 &max ) const;
  
      /// \brief Get the geom's space ID
      public: dSpaceID GetSpaceId() const;
  
      /// \brief Set the geom's space ID
      public: void SetSpaceId(dSpaceID spaceid);
  
      protected: dSpaceID spaceId;
  
      ///  ID for the sub-geom
      protected: dGeomID geomId;
    };
  
    /// \}
  
  }
}
#endif
