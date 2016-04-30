/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_DARTPHYSICS_PRIVATE_HH_
#define _GAZEBO_DARTPHYSICS_PRIVATE_HH_

#include "gazebo/physics/dart/dart_inc.h"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTPhysics
    class DARTPhysicsPrivate
    {
      /// \brief Constructor
      public: DARTPhysicsPrivate()
        : dtWorld(new dart::simulation::World())
      {
      }

      /// \brief Default destructor
      public: ~DARTPhysicsPrivate()
      {
      }

      /// \brief Disabled copy constructor
      public: DARTPhysicsPrivate(const DARTPhysicsPrivate&) = delete;

      /// \brief Pointer to DART World associated with this DART Physics.
      public: dart::simulation::WorldPtr dtWorld;
    };
  }
}
#endif
