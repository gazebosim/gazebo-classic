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

#include "gazebo/ecs/Manager.hh"
#include "gazebo/ecs/Entity.hh"

using namespace gazebo::ecs;


/// \brief Private class for PIMPL
class gazebo::ecs::EntityPrivate
{
  // TODO weak ptr?
  /// \brief The manager that created this entity
  public: Manager *manager;
};


EntityId Entity::nextId = 0;

/////////////////////////////////////////////////
// TODO manager to allocate id
Entity::Entity(Manager *_mgr)
: id(nextId++)
{
  this->impl.reset(new EntityPrivate());
  this->impl->manager = _mgr;
}

/////////////////////////////////////////////////
Entity::~Entity()
{
}

/////////////////////////////////////////////////
EntityId Entity::Id() const
{
  return this->id;
}

/////////////////////////////////////////////////
void *Entity::Component(const ComponentType &_type)
{
  return this->impl->manager->EntityComponent(this->id, _type);
}

/////////////////////////////////////////////////
void *Entity::AddComponent(const ComponentType &_type)
{
  return this->impl->manager->AddComponent(_type, this->id);
}

/////////////////////////////////////////////////
bool Entity::Matches(const std::set<ComponentType> &_types) const
{
  return this->impl->manager->EntityMatches(this->id, _types);
}
