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


#ifndef GAZEBO_ECS_CORE_ENTITYQUERYRESULT_HH_
#define GAZEBO_ECS_CORE_ENTITYQUERYRESULT_HH_

#include <memory>
#include "gazebo/ecs_core/Entity.hh"

namespace gazebo
{
namespace ecs_core
{

/// \brief Forward declaration
class EntityQueryResultPrivate;

/// \brief Forward declaration
class EntityManager;

class EntityQueryResult
{
  public:
    EntityQueryResult();
    ~EntityQueryResult();

    /// \brief number of entities that matched the query
    std::size_t NumResults() const;

    /// \brief Get the entity at the index in the results
    Entity At(std::size_t _index) const;

  private:
    friend EntityManager;

    std::shared_ptr<EntityQueryResultPrivate> impl;
};

}
}

#endif
