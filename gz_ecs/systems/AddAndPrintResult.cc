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
#include <iostream>
#include "gazebo/components/Triplet.hh"
#include "gazebo/ecs_core/Entity.hh"
#include "gazebo/ecs_core/EntityQuery.hh"
#include "gazebo/ecs_core/EntityQueryResult.hh"
#include "gazebo/ecs_core/EntityManager.hh"
#include "gazebo/plugin/RegisterMacros.hh"
#include "gazebo/private/systems/AddAndPrintResult.hh"

namespace gazebo
{
namespace systems
{

void AddAndPrintResult::Init(ecs_core::EntityQuery &_query)
{
  // Add components which are required
  _query.AddComponent<components::Triplet>();
}

void AddAndPrintResult::Update(
    double _dt, const ecs_core::EntityQueryResult &_result)
{
  // Loop through all of the entities which have the required components
  auto em = this->GetEntityManager();
  for (int i = 0; i < _result.NumResults(); i++)
  {
    ecs_core::Entity entity = _result.At(i);
    auto numbers = em->GetComponent<components::Triplet>(entity);

    std::cout << "Adding " << entity << ":" << 
      numbers->first + numbers->second + numbers->third << std::endl;
  }
}

}
}

GZ_REGISTER_SINGLE_PLUGIN(gazebo::systems::AddAndPrintResult,
                          gazebo::ecs_core::System)
