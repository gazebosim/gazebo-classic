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


#ifndef GAZEBO_PRIVATE_ECS_CORE_ENTITYMANAGERPRIVATE_HH_
#define GAZEBO_PRIVATE_ECS_CORE_ENTITYMANAGERPRIVATE_HH_

#include <unordered_map>
#include <vector>
#include <memory>

#include "gazebo/ecs_core/Entity.hh"
#include "gazebo/ecs_core/EntityQuery.hh"

namespace gazebo
{
namespace ecs_core
{

class EntityManagerPrivate
{
  public:
    /// \brief Id of the next Entity to create
    Entity nextEntity = 1;

    /// \brief Component by entity storage
    ///
    /// Outer map matches to an entity
    /// Inner map matches a typeid().hash_code of the component type to a 
    /// vector of bytes
    /// I'm not sure what kind of storage for components and entities will be
    /// On one hand, systems access the components of an entity at the same
    /// time so storing the components of an entity adjacent to each other
    /// seems like it would be cache-efficient.
    /// On the other hand most systems may only access one type of component
    /// so storing components of the same types tightly might reduce the
    /// number of cache misses.
    /// Artemis
    ///   stores components of the same type adjacent to each other
    /// Anax
    ///   stores components all over memory, but entities are a class that
    ///   have pointers to every component belonging to the entity.
    /// EntityX
    //    Stores components of the same type "semi-contiguous" in memory
    /// This is a prototype and right now easy is important
    /// TODO profile performance of component storage
    std::unordered_map<Entity, std::unordered_map<std::size_t, std::shared_ptr<char> > > entities;

    /// \brief Queries on this entity manager
    std::vector<EntityQuery> queries;
};

}
}

#endif
