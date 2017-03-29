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

#ifndef GAZEBO_PRIVATE_SYSTEMS_ADDANDPRINTRESULT_HH_
#define GAZEBO_PRIVATE_SYSTEMS_ADDANDPRINTRESULT_HH_

#include "gazebo/ecs/System.hh"

namespace gazebo
{
  namespace systems
  {
    /// \brief Forward Declaration
    class EntityQueryResult;

    class AddAndPrintResult : public ecs::System
    {
      public: virtual ecs::EntityQuery Init();

      public: virtual void Update(double _dt, ecs::EntityQuery &_result,
                  ecs::Manager &_mgr);
    };
  }
}
#endif
