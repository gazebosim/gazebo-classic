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

#ifndef GAZEBO_ECS_SYSTEM_HH_
#define GAZEBO_ECS_SYSTEM_HH_

#include <memory>
// #include "gazebo/ecs/Manager.hh"
// #include "gazebo/ecs/Component.hh"

namespace gazebo
{
  namespace ecs
  {
    /// \brief Forward declaration
    class SystemPrivate;

    class Manager;

    /// \brief Forward declaration
    class EntityQuery;

    /// \brief base class for a System
    ///
    /// A System operates on entities that have certain components. A system
    /// will only operate on an entity if it has all of the required components
    class System
    {
      public: System();
      public: ~System();

      /// \brief Initialize the system and return the entity query
      /// required by this system.
      public: virtual EntityQuery Init() = 0;

      // TODO ECS to put manager into dataPtr
        /// \brief Update all entities matching this system's requirements
      public: virtual void Update(
                  double _dt, EntityQuery &_result, Manager &_mgr) = 0;

      /// \brief No copy constructor
      private: System(const System&) = delete;

      private: std::unique_ptr<SystemPrivate> dataPtr;
    };
  }
}
#endif
