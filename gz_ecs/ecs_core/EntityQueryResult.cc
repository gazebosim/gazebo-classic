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

#include "gazebo/ecs_core/EntityQueryResult.hh"
#include "gazebo/ecs_core/EntityQueryResultPrivate.hh"

namespace gazebo
{
namespace ecs_core
{

EntityQueryResult::EntityQueryResult()
{
  this->impl.reset(new EntityQueryResultPrivate());
}

EntityQueryResult::~EntityQueryResult()
{
}

std::size_t EntityQueryResult::NumResults() const
{
  return this->impl->results.size();
}


Entity EntityQueryResult::At(std::size_t _index) const
{
  if (_index >= 0 && _index < this->NumResults())
  {
    return this->impl->results[_index];
  }
  return NO_ENTITY;
}

}
}
