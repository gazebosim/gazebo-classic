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
 
  class Geom; 
  class Body;
  class Model;

  class OgreVisual;
  /// \addtogroup gazebo_server
  /// \{
  
  
  /// Base class for all objects in Gazebo
  /*
   * Facilitates meshing of physics engine with rendering engine
   */
  class Entity : public Common
  {
    public: enum Type{DEFAULT, MODEL, BODY, GEOM, LIGHT};

    /// \brief Constructor
    /// \param parent Parent of the entity.
    public: Entity(Entity *parent = NULL);
  
    /// \brief Destructor
    public: virtual ~Entity();

    /// \brief Return the ID of the parent
    /// \return Integer ID
    public: int GetParentId() const;
  
    /// \brief Set the parent
    /// \param parent Parent entity
    public: void SetParent(Entity *parent);
  
    /// \brief Get the parent
    /// \return Pointer to the parent entity
    public: Entity *GetParent() const;

    /// \brief Add a child to this entity
    /// \param child Child entity
    public: void AddChild(Entity *child);

    /// \brief Remove a child from this entity
    /// \param child Child to remove
    public: virtual void RemoveChild(Entity *child);
  
    /// \brief Get all children
    /// \return Vector of children entities
    public: const std::vector< Entity* >  &GetChildren() const;

    /// \brief Get the number of children
    public: unsigned int GetChildCount() const;

    /// \brief Get a child by index
    public: Entity *GetChild(unsigned int i);

    /// \brief Get a child by name
    public: Entity *GetChild(const std::string &name );
  
    /// \brief Return this entity's sceneNode
    /// \return Ogre scene node
    public: OgreVisual *GetVisualNode() const;
  
    /// \brief Set the scene node
    /// \param sceneNode Ogre scene node
    public: void SetVisualNode(OgreVisual *visualNode);
 
    /// \brief Set whether this entity is static: immovable
    /// \param s Bool, true = static
    public: void SetStatic(const bool &s);
  
    /// \brief Return whether this entity is static
    /// \return bool True = static
    public: bool IsStatic() const;
  
    /// \brief Set whether this entity has been selected by the user through 
    //         the gui
    public: virtual bool SetSelected( bool s );
  
    /// \brief True if the entity is selected by the user
    public: bool IsSelected() const;

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

    public: void Print(std::string prefix);

    /// \brief Returns true if the entities are the same. Checks only the name
    public: bool operator==(const Entity &ent) const;

    /// \brief Get the parent model, if one exists
    /// \return Pointer to a model, or NULL if no parent model exists
    public: Model *GetParentModel() const;

    /// \brief Return the name of this entity with the model scope
    ///        model1::...::modelN::entityName
    public: std::string GetScopedName() const;

    /// \brief Return the name of this entity with the model+body+geom scope
    ///        model1::...::modelN::bodyN::entityName
    public: std::string GetCompleteScopedName() const;

    // TODO: THIS IS A HACK AND SHOULD BE REMOVED
    public: std::string GetCompleteScopedAltName() const;

    /// \brief Set the type of this entity
    public: void SetType(Type type);

    /// \brief Get the type of this entity
    public: Type GetType() const;

    /// \brief Get the type as a string
    public: std::string GetTypeString() const;

    /// \brief Handle a change of pose
    private: void PoseChange(bool notify = true);

    /// \brief Parent of this entity
    protected: Entity *parent;
  
    /// \brief Children of this entity
    protected: std::vector< Entity* > children;
  
    // is this an static entity
    protected: ParamT<bool> *staticP;
  
    /// \brief Visual stuff
    protected: OgreVisual *visualNode;
  
    private: bool selected;

    private: Pose3d relativePose;

    protected: Type type;
  };
  
  /// \}
}

#endif
