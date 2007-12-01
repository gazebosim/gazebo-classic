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

#include <Ogre.h>
#include <ode/ode.h>

#include "Entity.hh"
#include "Pose3d.hh"
#include "Vector3.hh"

/*namespace Ogre
{
  class Material;
  class Entity;
  class MovableObject;
  class SceneNode;
}*/

namespace gazebo
{

  class Body;
  class ContactParams;
  class XMLConfigNode;

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
  
    /// \brief Attach a mesh to the geom
    /// \param meshName Name of the mesh
    public: void AttachMesh(const std::string &meshName);
  
    /// \brief Attach a moveable object to the node
    /// \param obj The moveable object
    public: void AttachObject( Ogre::MovableObject *obj );
  
    /// \brief Set the scale of the mesh
    /// \param scale New scale of the mesh
    public: void ScaleMesh(const Vector3 &scale);
  
    /// \brief Set whether the mesh casts shadows
    /// \param enable True=cast shadows
    public: void SetCastShadows(bool enable);
  
    /// \brief Set the material to apply to the mesh
    public: void SetMeshMaterial(const std::string &materialName);
  
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

    /// \brief Set the transparency
    public: void SetTransparency( float trans );

    /// \brief Get the value of the transparency
    public: float GetTransparency() const;
  
    ///  Contact parameters
    public: ContactParams *contact; 
  
    /// The body this geom belongs to
    protected: Body *body;
  
    private: bool placeable;

    ///  Ogre Object
    protected: Ogre::MovableObject *ogreObj;

    ///  ODE object
    protected: Ogre::MovableObject *odeObj;
  
    /// ID for the transform geom
    protected: dGeomID transId;

    ///  ID for the sub-geom
    protected: dGeomID geomId;
  
    private: static int geomIdCounter;
  
    ///  Mass of this geometry
    protected: dMass mass;

    /// mass of the body
    protected: dMass bodyMass;
 
    ///  Extra rotation, used for cylinders
    protected: Quatern extraRotation;
  
    private: int laserFiducialId;
    private: float laserRetro;

    ///  name of the mesh
    protected: std::string meshName;

    ///  Mass as a double
    protected: double dblMass;

    private: Ogre::SceneNode *boundingBoxNode;

    private: Ogre::MaterialPtr origMaterial;
    private: Ogre::MaterialPtr myMaterial;
    private: Ogre::SceneBlendType sceneBlendType;

    private: float transparency;
  };

  /// \}

}
#endif
