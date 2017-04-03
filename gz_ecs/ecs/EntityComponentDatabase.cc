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

#include <algorithm>
#include <utility>

#include "gazebo/ecs/EntityComponentDatabase.hh"
#include "gazebo/ecs/EntityQuery.hh"


using namespace gazebo::ecs;

class gazebo::ecs::EntityComponentDatabasePrivate
{
  /// \brief instances of entities
  public: std::vector<Entity> entities;

  // TODO better storage of components
  // Map EntityId/ComponentType pair to an index in this->components
  public: std::map<std::pair<EntityId, ComponentType>, int> componentIndices;
  public: std::vector<char*> components;

  /// \brief update queries because this entity's components have changed
  public: void UpdateQueries(EntityId _id);

  /// \brief check if an entity has these components
  /// \returns true iff entity has all components in the set
  public: bool EntityMatches(EntityId _id,
              const std::set<ComponentType> &_types) const;

  /// \brief Queries on this manager
  public: std::vector<EntityQuery> queries;
};

/////////////////////////////////////////////////
EntityComponentDatabase::EntityComponentDatabase() :
  impl(new EntityComponentDatabasePrivate)
{
}

/////////////////////////////////////////////////
EntityComponentDatabase::~EntityComponentDatabase()
{
  // Call destructor on the components
  for (auto const &kv : this->impl->componentIndices)
  {
    const ComponentType &type = kv.first.second;
    const int index = kv.second;

    char *storage = this->impl->components[index];
    void *data = static_cast<void*>(storage);

    ComponentTypeInfo info = ComponentFactory::TypeInfo(type);
    info.destructor(data);
    delete [] storage;
  }
}

/////////////////////////////////////////////////
bool EntityComponentDatabase::AddQuery(const EntityQuery &_query)
{
  bool isDuplicate = false;
  for (auto const &query : this->impl->queries)
  {
    if (query == _query)
    {
      // Already have this query, bail
      isDuplicate = true;
      break;
    }
  }
  if (!isDuplicate)
  {
    this->impl->queries.push_back(_query);
    auto &nonConstQuery = this->impl->queries.back();
    auto const types = _query.ComponentTypes();
    for (int id = 0; id < this->impl->entities.size(); ++id)
    {
      if (this->impl->EntityMatches(id, types))
      {
        nonConstQuery.AddEntity(id);
      }
    }
  }
  return !isDuplicate;
}

bool EntityComponentDatabase::RemoveQuery(EntityQuery &_query)
{
  _query.Clear();
  std::remove(this->impl->queries.begin(), this->impl->queries.end(), _query);
}

/////////////////////////////////////////////////
EntityId EntityComponentDatabase::CreateEntity()
{
  // TODO Reuse deleted entity ids
  EntityId id = this->impl->entities.size();
  this->impl->entities.push_back(gazebo::ecs::Entity(this, id));
  return id;
}

/////////////////////////////////////////////////
gazebo::ecs::Entity EntityComponentDatabase::Entity(EntityId _id) const
{
  if (_id >= 0 && _id < this->impl->entities.size())
    return this->impl->entities[_id];
  else
  {
    return gazebo::ecs::Entity(
      const_cast<EntityComponentDatabase*>(this), NO_ENTITY);
  }
}

/////////////////////////////////////////////////
void *EntityComponentDatabase::AddComponent(EntityId _id, ComponentType _type)
{
  void *component = nullptr;
  auto key = std::make_pair(_id, _type);
  if (this->impl->componentIndices.find(key) ==
      this->impl->componentIndices.end())
  {
    // Allocate memory and call constructor
    ComponentTypeInfo info = ComponentFactory::TypeInfo(_type);
    // TODO store components of same type in adjacent memory
    char *storage = new char[info.size];
    component = static_cast<void *>(storage);
    info.constructor(component);

    auto index = this->impl->components.size();
    this->impl->componentIndices[key] = index;
    this->impl->components.push_back(storage);
    this->impl->UpdateQueries(_id);
  }

  return component;
}

/////////////////////////////////////////////////
void *EntityComponentDatabase::EntityComponent(EntityId _id,
    ComponentType _type) const
{
  void *component = nullptr;
  auto key = std::make_pair(_id, _type);
  if (this->impl->componentIndices.find(key) !=
      this->impl->componentIndices.end())
  {
    auto index = this->impl->componentIndices[key];
    char *data = this->impl->components[index];
    component = static_cast<void*>(data);
  }
  return component;
}

/////////////////////////////////////////////////
bool EntityComponentDatabasePrivate::EntityMatches(EntityId _id,
    const std::set<ComponentType> &_types) const
{
  for (auto const &type : _types)
  {
    auto key = std::make_pair(_id, type);
    if (this->componentIndices.find(key) == this->componentIndices.end())
    {
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////
void EntityComponentDatabasePrivate::UpdateQueries(EntityId _id)
{
  for (auto &query : this->queries)
  {
    if (this->EntityMatches(_id, query.ComponentTypes()))
      query.AddEntity(_id);
  }
}
