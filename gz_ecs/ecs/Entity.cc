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

using namespace gazebo;
using namespace ecs;

EntityId Entity::nextId = 0;

/////////////////////////////////////////////////
Entity::Entity(Manager *_mgr)
: manager(_mgr), id(nextId++)
{
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
ComponentBase *Entity::ComponentBaseValue(const std::string &_comp)
{
  return this->manager->EntityComponent(this->id,
      ComponentFactory::Type(_comp));
}
