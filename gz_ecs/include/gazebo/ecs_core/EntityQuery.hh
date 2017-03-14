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


#ifndef GAZEBO_ECS_CORE_ENTITYQUERY_HH_
#define GAZEBO_ECS_CORE_ENTITYQUERY_HH_

#include <typeinfo>
#include <memory>
#include <vector>
#include "gazebo/ecs_core/EntityQueryResult.hh"

namespace gazebo
{
namespace ecs_core
{

/// \brief Forward declaration
class EntityQueryPrivate;

/// \brief a Class for querying entities from a manager
class EntityQuery
{
  public:
    EntityQuery();
    ~EntityQuery();

    template <typename T>
    void AddComponent()
    {
      this->AddComponent(typeid(T).hash_code());
    }

    /// \brief returns true if these are the same instance
    bool operator==(const EntityQuery &_rhs) const;

    /// \brief Returns the results of the query
    EntityQueryResult* Results() const;

    /// \brief Get the components that have been added to the query
    std::vector<std::size_t> ComponentTypes() const;

  private:

    void AddComponent(std::size_t _hash);

    std::shared_ptr<EntityQueryPrivate> impl;
};

}
}

#endif
