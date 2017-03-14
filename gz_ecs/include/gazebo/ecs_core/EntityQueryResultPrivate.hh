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


#ifndef GAZEBO_ECS_CORE_ENTITYQUERYRESULTPRIVATE_HH_
#define GAZEBO_ECS_CORE_ENTITYQUERYRESULTPRIVATE_HH_

#include <vector>
#include "gazebo/ecs_core/Entity.hh"

namespace gazebo
{
namespace ecs_core
{

class EntityQueryResultPrivate
{
  public:
    /// \brief all entities that matched the query
    std::vector<Entity> results;

};

}
}

#endif
