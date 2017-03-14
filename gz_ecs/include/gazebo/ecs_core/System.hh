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


#ifndef GAZEBO_ECS_CORE_SYSTEM_HH_
#define GAZEBO_ECS_CORE_SYSTEM_HH_

#include <memory>

namespace gazebo
{
namespace ecs_core
{

/// \brief Forward declaration
class SystemPrivate;

/// \brief Forward declaration
class SystemManager;

/// \brief Forward declaration
class EntityManager;

/// \brief Forward declaration
class EntityQuery;

/// \brief Forward declaration
class EntityQueryResult;

/// \brief base class for a System
///
/// A System operates on entities that have certain components. A system
/// will only operate on an entity if it has all of the required components
class System
{
  public:
    System();
    ~System();

    /// \brief Start the system in a system manager
    virtual void Init(EntityQuery &_query) = 0;

    /// \brief Update all entities matching this system's requirements
    virtual void Update(
        double _dt, const EntityQueryResult &_result) = 0;

  protected:
    /// \brief Get the SystemManager this system is attached to
    SystemManager* GetSystemManager() const;

    /// \brief Get the EntityManager this system should be using
    EntityManager* GetEntityManager() const;

  private:
    /// \brief SystemManager needs to know what components are required
    friend SystemManager;

    /// \brief No copy constructor
    System(const System&) = delete;

    std::unique_ptr<SystemPrivate> impl;
};

}
}

#endif
