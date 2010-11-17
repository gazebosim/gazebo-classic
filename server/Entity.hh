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
/* Desc: Base class for all physical entities
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 * SVN: $Id$
 */

#ifndef ENTITY_HH
#define ENTITY_HH

#include <vector>
#include <string>

#include "Common.hh"
#include "Pose3d.hh"
#include "Param.hh"

namespace gazebo
{

  class Visual;
  /// \addtogroup gazebo_server
  /// \{
  
  /// Base class for all objects in Gazebo
  /*
   * Facilitates meshing of physics engine with rendering engine
   */
  class Entity : public Common
  {

    /// \brief Constructor
    /// \param parent Parent of the entity.
    public: Entity(Common *parent = NULL);
  
    /// \brief Destructor
    public: virtual ~Entity();
 
    public: void SetName(const std::string &name);
 
    /// \brief Return this entity's sceneNode
    /// \return Ogre scene node
    public: Visual *GetVisualNode() const;
  
    /// \brief Set the scene node
    /// \param sceneNode Ogre scene node
    public: void SetVisualNode(Visual *visualNode);
 
    /// \brief Set whether this entity is static: immovable
    /// \param s Bool, true = static
    public: void SetStatic(const bool &s);
  
    /// \brief Return whether this entity is static
    /// \return bool True = static
    public: bool IsStatic() const;
  
    /// \brief Get the absolute pose of the entity
    public: virtual Pose3d GetWorldPose() const;

    /// \brief Get the pose of the entity relative to its parent
    public: Pose3d GetRelativePose() const;

    /// \brief Get the pose relative to the model this entity belongs to
    public: Pose3d GetModelRelativePose() const;

    /// \brief Set the pose of the entity relative to its parent
    public: void SetRelativePose(const Pose3d &pose, bool notify = true);

    /// \brief Set the abs pose of the entity
    public: void SetWorldPose(const Pose3d &pose, bool notify=true);

    /// \brief Set the position of the entity relative to its parent
    public: void SetRelativePosition(const Vector3 &pos);

    /// \brief Set the rotation of the entity relative to its parent
    public: void SetRelativeRotation(const Quatern &rot);


    /// \brief Get the linear velocity of the entity
    public: virtual Vector3 GetRelativeLinearVel() const
            {return Vector3();}

    /// \brief Get the linear velocity of the entity in the world frame
    public: virtual Vector3 GetWorldLinearVel() const
            {return Vector3();}


    /// \brief Get the angular velocity of the entity
    public: virtual Vector3 GetRelativeAngularVel() const
            {return Vector3();}

    /// \brief Get the angular velocity of the entity in the world frame
    public: virtual Vector3 GetWorldAngularVel() const
            {return Vector3();}


    /// \brief Get the linear acceleration of the entity
    public: virtual Vector3 GetRelativeLinearAccel() const
            {return Vector3();}

    /// \brief Get the linear acceleration of the entity in the world frame
    public: virtual Vector3 GetWorldLinearAccel() const
            {return Vector3();}


    /// \brief Get the angular acceleration of the entity 
    public: virtual Vector3 GetRelativeAngularAccel() const
            {return Vector3();}

    /// \brief Get the angular acceleration of the entity in the world frame
    public: virtual Vector3 GetWorldAngularAccel() const
            {return Vector3();}


    /// \brief This function is called when the entity's (or one of its parents)
    ///        pose of the parent has changed
    protected: virtual void OnPoseChange() {}

    /// \brief Handle a change of pose
    private: void PoseChange(bool notify = true);

    // is this an static entity
    protected: ParamT<bool> *staticP;
  
    /// \brief Visual stuff
    protected: Visual *visualNode;
  

    private: Pose3d relativePose;
  };
  
  /// \}
}

#endif
