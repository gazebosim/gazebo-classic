/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef GAZEBO_ECS_ENTITYCOMPONENTDATABASE_HH_
#define GAZEBO_ECS_ENTITYCOMPONENTDATABASE_HH_

#include <memory>

#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/ComponentFactory.hh"

namespace gazebo
{
namespace ecs
{

/// \brief Forward declaration
class EntityQuery;

/// \brief Forward declaration
class EntityComponentDatabasePrivate;

/// \brief Stores and retrieves entities/components efficiently
///
/// This class stores entities and components, and provides efficient
/// queries for retrieving them.
class EntityComponentDatabase
{
  /// \brief Constructor
  public: EntityComponentDatabase();

  /// \brief Destructor
  public: ~EntityComponentDatabase();

  /// \brief Add a query for entities
  /// \returns true if query was successfully added
  public: bool AddQuery(const EntityQuery &_query);

  /// \brief remove a query for entities
  /// \returns true if query was successfully removed
  public: bool RemoveQuery(EntityQuery &_query);

  /// \brief Creates a new entity
  /// \brief returns an id for the entity, or NO_ENTITY on failure
  public: EntityId CreateEntity();

  /// \brief Get an Entity instance by Id
  public: ::gazebo::ecs::Entity Entity(EntityId _id) const;

  // TODO templated version
  /// \brief Add a new component to an entity
  public: void *AddComponent(EntityId _id, ComponentType _type);

  // TODO templated version
  /// \brief Get a component that's on an entity
  public: void *EntityComponent(EntityId _id, ComponentType _type) const;

  /// \brief Private IMPLementation pointer
  private: std::shared_ptr<EntityComponentDatabasePrivate> impl;
};

}
}

#endif
