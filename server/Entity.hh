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

/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 * SVN: $Id$
 */

#ifndef ENTITY_HH
#define ENTITY_HH

#include <vector>
#include <string>
#include <ode/ode.h>

namespace gazebo
{

class OgreVisual;
/// \addtogroup gazebo_server
/// \{


/// Base class for all objects in Gazebo
/*
 * Facilitates meshing of physics engine with rendering engine
 */
class Entity
{
  /// \brief Constructor
  /// \param parent Parent of the entity.
  public: Entity(Entity *parent = NULL);

  /// \brief Destructor
  public: virtual ~Entity();

  /// \brief Return the ID of this entity. This id is unique
  /// \return Integer ID
  public: int GetId() const;

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

  /// \brief Get all children
  /// \return Vector of children entities
  public: std::vector< Entity* > &GetChildren();

  /// \brief Return this entity's sceneNode
  /// \return Ogre scene node
  public: OgreVisual *GetVisualNode() const;

  /// \brief Set the scene node
  /// \param sceneNode Ogre scene node
  public: void SetVisualNode(OgreVisual *visualNode);

  /// \brief Set the name of the entity
  /// \param name Body name
  public: void SetName(const std::string &name);

  /// \brief Return the name of the entity
  /// \return Name of the entity
  public: std::string GetName() const;

  /// \brief Set whether this entity is static: immovable
  /// \param s Bool, true = static
  public: void SetStatic(bool s);

  /// \brief Return whether this entity is static
  /// \return bool True = static
  public: bool IsStatic() const;

  /// \brief Returns true if the entities are the same. Checks only the name
  public: bool operator==(const Entity &ent);

  /// \brief Parent of this entity
  protected: Entity *parent;

  /// \brief Children of this entity
  public: std::vector< Entity* > children;

  /// \brief This entities ID
  private: unsigned int id;

  /// \brief Used to automaticaly chose a unique ID on creation
  private: static unsigned int idCounter;

  /// \brief OGRE stuff
  protected: OgreVisual *visualNode;

  /// \brief ODE Stuff
  public: dSpaceID spaceId;

  /// \brief Name of the entity
  private: std::string name;

  private: bool isStatic;
};

/// \}
}

#endif
