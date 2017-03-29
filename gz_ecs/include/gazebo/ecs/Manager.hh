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

#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/System.hh"
#include "gazebo/ecs/Component.hh"
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

      /// \brief Updates all the queryies which have been added to it
      public: void UpdateEntities();

      public: void UpdateSystems(const double _dt);

      public: Entity &GetEntity(const EntityId _id) const;

      public: template <typename T>
              T &AddComponent(const std::string &_name, EntityId _id)
              {
                ComponentPtr<T> cmp = ComponentFactory::Create<T>(_name);
                ComponentId id = cmp->Id();
                this->AddComponent(std::move(cmp), _id);
                return static_cast<Component<T>*>(
                    this->EntityComponent(id))->data;
              }

      public: void AddComponent(std::unique_ptr<ComponentBase> _cmp,
                                EntityId _id);

      public: void UpdateSystem(double _dt);

      public: void UpdateEntity(const EntityId _id);

      private: ComponentBase *EntityComponent(const ComponentId _compId);
      private: ComponentBase *EntityComponent(const EntityId _id,
                                             const ComponentType _compType);

      /// \brief Add a query for components of a certain type
      ///
      /// This does the query, and returns a proxy to the result. The result
      /// Can be used to retreive the entities that matched. The query will
      /// Continue to be processed until it is removed.
      private: std::size_t AddQuery(const EntityQuery &_query);

      private: Manager(const Manager&) = delete;

      private: std::unique_ptr<ManagerPrivate> dataPtr;

      private: friend class Entity;
    };
  }
}
#endif
