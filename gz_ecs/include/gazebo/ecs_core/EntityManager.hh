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


#ifndef GAZEBO_ECS_CORE_ENTITYMANAGER_HH_
#define GAZEBO_ECS_CORE_ENTITYMANAGER_HH_

#include <memory>
#include <typeinfo>


#include "gazebo/ecs_core/Entity.hh"
#include "gazebo/ecs_core/EntityQuery.hh"

namespace gazebo
{
namespace ecs_core
{

/// \brief Forward declaration
class EntityManagerPrivate;

/// \brief Holds entities and components
///
/// This class holds all the entities and components. It can be queried
/// for all entities that match components. It's becomming clear to me
/// that the performance of an ECS system is largely determined by how
/// the entity manager works. I think there will be big benefits if this
/// class can cache query results and store components in a way to avoid
/// cache-misses.
class EntityManager
{
  public:
    EntityManager();
    ~EntityManager();

    Entity CreateEntity();

    template <typename T>
    T* AddComponent(Entity _id)
    {
      return static_cast<T*>(this->AddComponent(_id, typeid(T).hash_code(), sizeof(T)));
    }

    template <typename T>
    T* GetComponent(Entity _id)
    {
      return static_cast<T*>(this->GetComponent(_id, typeid(T).hash_code(), sizeof(T)));
    }

    /// \brief Add a query for components of a certain type
    /// 
    /// This does the query, and returns a proxy to the result. The result
    /// Can be used to retreive the entities that matched. The query will
    /// Continue to be processed until it is removed.
    bool AddQuery(const EntityQuery &_query);

    /// \brief Stops processing of a query
    ///
    /// This removes the query from an entity manager. It will stop being
    /// updated with new data
    void RemoveQuery(const EntityQuery &_query);

    /// \brief Updates all the queryies which have been added to it
    void Update();

  private:
    void* AddComponent(Entity _id, std::size_t _hash, std::size_t _size);

    void* GetComponent(Entity _id, std::size_t _hash, std::size_t _size);

    EntityManager(const EntityManager&) = delete;

    std::unique_ptr<EntityManagerPrivate> impl;

};

}
}

#endif
