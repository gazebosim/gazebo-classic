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

#ifndef GAZEBO_ECS_ENTITY_HH_
#define GAZEBO_ECS_ENTITY_HH_

#include <map>
#include <set>
#include <vector>
#include "gazebo/ecs/Component.hh"

namespace gazebo
{
  namespace ecs
  {
    /// \brief An entity is an id!
    typedef int EntityId;

    /// \brief For results which there is no entity
    const EntityId NO_ENTITY = -1;

    class Manager;
    class Entity
    {
      public: Entity(Manager *mgr);
      public: ~Entity();

      public: EntityId Id() const;

      public: template <typename T>
              T &ComponentValue(const std::string &_comp)
              {
                return static_cast<Component<T> *>(
                    this->ComponentBaseValue(_comp))->data;
              }

      public: ComponentId CmpId(const ComponentType &_type) const
              {
                return this->typeIds.find(_type)->second;
              }

      public: ComponentType CmpType(const ComponentId &_id) const
              {
                return this->idTypes.find(_id)->second;
              }

      public: void AddComponent(const ComponentId _id,
                                const ComponentType _type)
              {
                this->typeIds[_type] = _id;
                this->idTypes[_id] = _type;
              }

      public: bool Matches(const std::set<ComponentType> &_types) const
              {
                for (auto const &t : _types)
                {
                  if (this->typeIds.find(t) == this->typeIds.end())
                  {
                    return false;
                  }
                }
                return true;
              }

      private: ComponentBase *ComponentBaseValue(const std::string &_comp);

      private: Manager *manager;
      private: EntityId id;
      private: static EntityId nextId;

      private: std::map<ComponentType, ComponentId> typeIds;
      private: std::map<ComponentId, ComponentType> idTypes;
    };
  }
}
#endif
