/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
#include "Param.hh"
#include "Entity.hh"
#include "Pose3d.hh"
#include "Vector3.hh"
#include "Geom.hh"

namespace gazebo
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
#endif
