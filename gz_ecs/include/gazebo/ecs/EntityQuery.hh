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

#ifndef GAZEBO_ECS_ENTITYQUERY_HH_
#define GAZEBO_ECS_ENTITYQUERY_HH_

#include <memory>
#include <vector>
#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/ComponentFactory.hh"

namespace gazebo
{
  namespace ecs
  {
    /// \brief Forward declaration
    class EntityQueryPrivate;

    /// \brief Forward declaration
    class EntityComponentDatabase;

    /// \brief a Class for querying entities from a manager
    class EntityQuery
    {
      public: EntityQuery();
      public: ~EntityQuery();

      // TODO templated version
      /// \brief Add a component based on a name.
      public: bool AddComponent(const std::string &_name);

      /// \brief Add a component based on a component type.
      public: void AddComponent(ComponentType _type);

      /// \brief Get the components that have been added to the query
      public: const std::set<ComponentType> &ComponentTypes() const;

      /// \brief returns true if these are the same instance
      public: bool operator==(const EntityQuery &_rhs) const;

      public: bool AddEntity(EntityId _id);

      /// \brief Get the entity ids
      /// TODO ordered results matching component placement in memory
      public: const std::set<EntityId> &EntityIds() const;

      /// \brief Clear results of a query
      private: void Clear();

      /// \brief friendship
      private: friend EntityComponentDatabase;

      private: std::shared_ptr<EntityQueryPrivate> dataPtr;
    };
  }
}
#endif
