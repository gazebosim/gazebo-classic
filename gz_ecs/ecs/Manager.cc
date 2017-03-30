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
#include <set>
#include <vector>
#include <unordered_map>

#include "gazebo/components/Fraction.hh"
#include "gazebo/components/Triplet.hh"

#include "gazebo/plugin/PluginLoader.hh"
#include "gazebo/ecs/EntityQuery.hh"
#include "gazebo/ecs/Manager.hh"

using namespace gazebo;
using namespace ecs;

/// \brief Forward declaration
class System;

class gazebo::ecs::ManagerPrivate
{
  /// \brief Systems that are added to the manager
  public: std::vector<
          std::pair<std::unique_ptr<System>, std::size_t>> systems;

  /// \brief Component by entity storage
  ///
  /// Outer map matches to an entity
  /// Inner map matches component type to bytes
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
  ///   Stores components of the same type "semi-contiguous" in memory
  /// This is a prototype and right now easy is important
  /// TODO profile performance of component storage


  // TODO Have a EntityManager and SystemManager

  // All the entities
  public: std::vector<Entity> entities;

  public: std::map<ComponentId, std::unique_ptr<ComponentBase>> components;

  /// \brief Queries on this manager
  public: std::vector<EntityQuery> queries;

  /// \brief Something to deal with loading plugins
  public: gazebo::plugin::PluginLoader pl;
};

/////////////////////////////////////////////////
Manager::Manager()
: dataPtr(new ManagerPrivate)
{
  ComponentFactory::Register<gazebo::components::Triplet>(
      "gazebo::components::Triplet");

  ComponentFactory::Register<gazebo::components::Fraction>(
      "gazebo::components::Fraction");
}

/////////////////////////////////////////////////
Manager::~Manager()
{
}

/////////////////////////////////////////////////
EntityId Manager::CreateEntity()
{
  // TODO Reuse ids, This will run out of entities eventually
  this->dataPtr->entities.push_back(Entity(this));
  std::cout << "Entity Size[" << this->dataPtr->entities.size() << "]\n";
  return this->dataPtr->entities.back().Id();
}

/////////////////////////////////////////////////
void Manager::UpdateEntities()
{
  // Update EntityQuery results
  for (auto &query : this->dataPtr->queries)
  {
    // Check every entity
    // TODO this could be faster if which entities had components added
    // or removed was tracked somehow, then the queries would only be
    // updated with those entities that changed
    for (auto const &entity : this->dataPtr->entities)
    {
      if (entity.Matches(query.ComponentTypes()))
      {
        //query.AddEntity(&entity);
        query.AddEntity(entity.Id());
      }
    }
  }
}

/////////////////////////////////////////////////
void Manager::UpdateEntity(const EntityId _id)
{
  // Update EntityQuery results
  for (auto &query : this->dataPtr->queries)
  {
    if (this->dataPtr->entities[_id].Matches(query.ComponentTypes()))
    {
      //query.AddEntity(&(this->dataPtr->entities[_id]));
      query.AddEntity(_id);
    }
  }
}

/////////////////////////////////////////////////
std::size_t Manager::AddQuery(const EntityQuery &_query)
{
  for (std::size_t i = 0; i < this->dataPtr->queries.size(); ++i)
  {
    if (this->dataPtr->queries[i] == _query)
    {
      // Already have this query, bail
      return i;
    }
  }

  this->dataPtr->queries.push_back(_query);
  return this->dataPtr->queries.size() - 1;
}

/////////////////////////////////////////////////
void Manager::UpdateSystems(const double _dt)
{
  // TODO There is a lot of opportunity for parallelization here
  // In general systems are run sequentially, one after the other
  //  Different Systems can run in parallel if they don't share components
  //  How to handle systems which add or remove entities?
  //  Defer creation and deletion?
  // Some systems could be run on multiple cores, such that several
  //  instances of a system each run on a subset of the entities returned
  //  in a query result.
  // Heck, entity and component data can be shared over the network to
  //  other SystemManagers to use multiple machines for one simulation

  // But this is a prototype, so here's the basic implementation
  for(auto &system : this->dataPtr->systems)
  {
    system.first->Update(_dt, this->dataPtr->queries[system.second], *this);
  }
}

/////////////////////////////////////////////////
bool Manager::LoadSystem(std::unique_ptr<System> _sys)
{
  bool success = false;
  if (_sys)
  {
    this->dataPtr->systems.push_back(
        std::make_pair(std::move(_sys), this->AddQuery(_sys->Init())));
    success = true;
  }
  return success;
}

/////////////////////////////////////////////////
void Manager::AddComponent(std::unique_ptr<ComponentBase> _cmp, EntityId _id)
{
  // Insert the type of the component into the entity component container.
  this->dataPtr->entities[_id].AddComponent(_cmp->Id(), _cmp->Type());

  // Store the pointer to the actual component.
  this->dataPtr->components[_cmp->Id()] = std::move(_cmp);

  this->UpdateEntity(_id);
}

/////////////////////////////////////////////////
ComponentBase *Manager::EntityComponent(const ComponentId _compId)
{
  return this->dataPtr->components[_compId].get();
}

/////////////////////////////////////////////////
ComponentBase *Manager::EntityComponent(const EntityId _id,
                               const ComponentType _compType)
{
  return this->dataPtr->components[
    this->dataPtr->entities[_id].CmpId(_compType)].get();
}

/////////////////////////////////////////////////
Entity &Manager::GetEntity(const EntityId _id) const
{
  return this->dataPtr->entities[_id];
}
