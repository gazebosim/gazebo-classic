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

#ifndef GAZEBO_ECS_MANAGER_HH_
#define GAZEBO_ECS_MANAGER_HH_

#include <memory>
#include <iostream>
#include <set>

#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/System.hh"
#include "gazebo/ecs/ComponentFactory.hh"

namespace gazebo
{
  namespace ecs
  {
    // Forward declare private data class.
    class ManagerPrivate;

    class Manager
    {
      public: Manager();
      public: ~Manager();

      public: EntityId CreateEntity();

      /// \brief Convenience function to load a system from a type
      ///
      /// Ex: sm->LoadSystem<FancySystemClass>();
      public: template <typename T>
              bool LoadSystem()
              {
                return this->LoadSystem(std::unique_ptr<System>(new T()));
              }

      /// \brief Load a system
      ///
      /// Ex: sm->LoadSystem(std::move(aUniquePtrInstance))
      public: bool LoadSystem(std::unique_ptr<System> _sys);

      public: void UpdateSystems(const double _dt);

      /// \brief Returns an entity instance with the given ID
      /// \returns Entity with id set to NO_ENTITY if entity does not exist
      public: gazebo::ecs::Entity Entity(const EntityId _id) const;

      public: template <typename T>
              T *AddComponent(EntityId _id)
              {
                ComponentType type = ComponentFactory::Type<T>();
                return static_cast<T*>(this->AddComponent(type, _id));
              }

      /// \brief Get component on entity by component type
      /// \returns pointer to component iff entity has component of that type
      public: void *EntityComponent(EntityId _id, ComponentType _type);

      /// \brief Add component to entity by ComponentType
      /// \returns pointer to component iff it was successfully added
      public: void *AddComponent(ComponentType _type, EntityId _id);

      public: void UpdateSystem(double _dt);

      /// \brief Add a query for components of a certain type
      /// \returns true if the query was added successfully
      ///
      /// The query will be processed until it is removed.
      private: bool AddQuery(const EntityQuery &_query);

      private: Manager(const Manager&) = delete;

      private: std::unique_ptr<ManagerPrivate> dataPtr;

      private: friend class Entity;
    };
  }
}
#endif
