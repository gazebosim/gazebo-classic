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


#ifndef GAZEBO_ECS_CORE_SYSTEMMANAGER_HH_
#define GAZEBO_ECS_CORE_SYSTEMMANAGER_HH_

#include <memory>

#include "gazebo/ecs_core/System.hh"

namespace gazebo
{
namespace ecs_core
{

/// \brief Forward Declaration
class SystemManagerPrivate;

/// \brief Forward Declaration
class EntityManager;

/// \brief A class responsible for keeping systems running
class SystemManager
{
  public:
    SystemManager();
    ~SystemManager();

    void SetEntityManager(EntityManager *_em);

    EntityManager* GetEntityManager();

    void Update(double _dt);

    template <typename T>
    void LoadSystem()
    {
      T *system = new T();
      this->LoadSystem(dynamic_cast<System*>(system));
    }

  private:
    /// \brief Load a system
    void LoadSystem(System *_sys);

    /// \brief No copy constructor
    SystemManager(const SystemManager&) = delete;

    std::unique_ptr<SystemManagerPrivate> impl;
};

}
}

#endif
