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

#include "gazebo/ecs_core/EntityQuery.hh"
#include "gazebo/ecs_core/EntityQueryPrivate.hh"


namespace gazebo
{
namespace ecs_core
{

EntityQuery::EntityQuery()
{
  this->impl.reset(new EntityQueryPrivate());
}

EntityQuery::~EntityQuery()
{
}

void EntityQuery::AddComponent(std::size_t _hash)
{
  auto begin = this->impl->componentTypes.begin();
  auto end = this->impl->componentTypes.end();
  if (end == std::find(begin, end, _hash))
  {
    this->impl->componentTypes.push_back(_hash);
  }
}

bool EntityQuery::operator==(const EntityQuery &_rhs) const
{
  // Copy constructor means shared_ptr will have the same address
  return this->impl == _rhs.impl;
}


EntityQueryResult* EntityQuery::Results() const
{
  return &(this->impl->results);
}

std::vector<std::size_t> EntityQuery::ComponentTypes() const
{
  return this->impl->componentTypes;
}

}
}
