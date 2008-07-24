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
 * SVN: $Id$
 */

#ifndef GEOM_HH
#define GEOM_HH

#include <ode/ode.h>

#include "Entity.hh"
#include "Pose3d.hh"
#include "Vector3.hh"

namespace gazebo
{

  class Body;
  class ContactParams;
  class XMLConfigNode;
  class OgreVisual;

  /// \addtogroup gazebo_physics
  /// \brief Base class for all geoms
  /// \{

  /// \brief Base class for all geoms
  class Geom : public Entity
  {
  
    /// \brief Constructor
    //public: Geom(Body *body, const std::string &name);
    public: Geom(Body *body);
  
    /// \brief Destructor
    public: virtual ~Geom();

    /// \brief Load the geom
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Load the geom
    public: virtual void Save();

    /// \brief Load child class
    protected: virtual void LoadChild(XMLConfigNode *node) = 0;
  
    /// \brief Set the encapsulated geometry object
    public: void SetGeom(dGeomID geomId, bool placeable);
  
    /// \brief Update function for geoms
    public: void Update();

    /// \brief Update child class
    public: virtual void UpdateChild() {};
  
    /// \brief Return the geom id
    /// \return The geom id
    public: dGeomID GetGeomId() const;
  
    /// \brief Return the transform id
    /// \return The transform id
    public: dGeomID GetTransId() const;
  
    /// \brief Get the ODE geom class
    public: int GetGeomClass() const;
  
    /// \brief Return whether this geom is placeable
    public: bool IsPlaceable() const;
  
    /// \brief Set the pose
    /// \param pose New pose
    /// \param updateCoM True to update the bodies Center of Mass
    public: void SetPose(const Pose3d &pose, bool updateCoM=true);
  
    /// \brief Return the pose of the geom
    public: Pose3d GetPose() const;
  
    /// \brief Set the position
    /// \param pos Vector3 position
    public: void SetPosition(const Vector3 &pos);
  
    /// \brief Set the rotation
    /// \param rot Quaternion rotation
    public: void SetRotation(const Quatern &rot);
  
    /// \brief Set the category bits, used during collision detection
    /// \param bits The bits
    public: void SetCategoryBits(unsigned int bits);
  
    /// \brief Set the collide bits, used during collision detection
    /// \param bits The bits
    public: void SetCollideBits(unsigned int bits);
  
    /// \brief Get the mass of the geom
    public: const dMass *GetBodyMassMatrix();
  
    /// \brief Set the laser fiducial integer id
    public: void SetLaserFiducialId(int id);
  
    /// \brief Get the laser fiducial integer id
    public: int GetLaserFiducialId() const;
  
    /// \brief Set the laser retro reflectiveness 
    public: void SetLaserRetro(float retro);
  
    /// \brief Get the laser retro reflectiveness 
    public: float GetLaserRetro() const;
 
    /// \brief Set the visibility of the bounding box
    public: void ShowBoundingBox(bool show);

    /// \brief Set the visibility of the joints
    public: void ShowJoints(bool show);

   /// \brief Set the visibility of the physical entity of this geom
    public: void ShowPhysics(bool);

    ///  Contact parameters
    public: ContactParams *contact; 
  
    /// The body this geom belongs to
    protected: Body *body;
  
    private: bool placeable;

    /// ID for the transform geom
    protected: dGeomID transId;

    ///  ID for the sub-geom
    protected: dGeomID geomId;
  
    private: static int geomIdCounter;
  
    ///  Mass of this geometry
    protected: dMass mass;

    /// mass of the body
    protected: dMass bodyMass;
 
    private: int laserFiducialId;
    private: float laserRetro;

    ///  Mass as a double
    protected: double dblMass;

    /// Special bounding box visual
    private: OgreVisual *bbVisual;

    private: float transparency;

    /// All the visual apparence 
    private: std::vector<OgreVisual*> visuals;

    ///our XML DATA
    private: XMLConfigNode *xmlNode;

  };

  /// \}

}
#endif
