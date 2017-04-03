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
#include <set>

#include "gazebo/ecs/EntityQuery.hh"

using namespace gazebo;
using namespace ecs;

class gazebo::ecs::EntityQueryPrivate
{
  /// \brief list of component types that must be present on entities
  public: std::set<ComponentType> componentTypes;

  /// \brief all entities that matched the query
  //public: std::vector<const Entity *> entities;

  public: std::set<EntityId> entityIds;
};

/////////////////////////////////////////////////
EntityQuery::EntityQuery()
: dataPtr(new EntityQueryPrivate())
{
}

/////////////////////////////////////////////////
EntityQuery::~EntityQuery()
{
}

/////////////////////////////////////////////////
bool EntityQuery::AddComponent(const std::string &_name)
{
  auto type = ComponentFactory::Type(_name);

  bool success = type != NO_COMPONENT;

  if (success)
    this->AddComponent(type);

  return success;
}

/////////////////////////////////////////////////
void EntityQuery::AddComponent(ComponentType _type)
{
  this->dataPtr->componentTypes.insert(_type);
}

/////////////////////////////////////////////////
bool EntityQuery::operator==(const EntityQuery &_rhs) const
{
  // Copy constructor means shared_ptr will have the same address
  return this->dataPtr == _rhs.dataPtr;
}

/////////////////////////////////////////////////
bool EntityQuery::AddEntity(EntityId _id)
{
  // Only add unique enities.
  if (this->dataPtr->entityIds.find(_id) == this->dataPtr->entityIds.end())
  {
    //this->dataPtr->entities.push_back(_entity);
    this->dataPtr->entityIds.insert(_id);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void EntityQuery::Clear()
{
  this->dataPtr->entityIds.clear();
}

/////////////////////////////////////////////////
const std::set<ComponentType> &EntityQuery::ComponentTypes() const
{
  return this->dataPtr->componentTypes;
}

/////////////////////////////////////////////////
const std::set<EntityId> &EntityQuery::EntityIds() const
{
  return this->dataPtr->entityIds;
}
