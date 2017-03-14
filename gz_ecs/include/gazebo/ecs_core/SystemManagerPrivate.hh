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


#ifndef GAZEBO_PRIVATE_ECS_CORE_SYSTEMMANAGERPRIVATE_HH_
#define GAZEBO_PRIVATE_ECS_CORE_SYSTEMMANAGERPRIVATE_HH_

#include <vector>
#include <memory>

namespace gazebo
{
namespace ecs_core
{

/// \brief Forward declaration
class EntityManager;

/// \brief Forward declaration
class System;


class SystemManagerPrivate
{
  public:
    EntityManager *entityManager;

    /// \brief Systems that are added to the manager
    std::vector<std::shared_ptr<System> > systems;

    /// \brief Queries for each system
    std::vector<EntityQuery> queries;
};

}
}

#endif
