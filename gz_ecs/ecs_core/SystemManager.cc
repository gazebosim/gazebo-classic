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


#include "gazebo/ecs_core/EntityManager.hh"
#include "gazebo/ecs_core/EntityQuery.hh"
#include "gazebo/ecs_core/SystemManager.hh"
#include "gazebo/ecs_core/SystemManagerPrivate.hh"
#include "gazebo/ecs_core/SystemPrivate.hh"


namespace gazebo
{
namespace ecs_core
{

SystemManager::SystemManager()
{
  this->impl.reset(new SystemManagerPrivate());
}

SystemManager::~SystemManager()
{
}

void SystemManager::SetEntityManager(EntityManager *_em)
{
  this->impl->entityManager = _em;
}

EntityManager* SystemManager::GetEntityManager()
{
  return this->impl->entityManager;
}

void SystemManager::Update(double _dt)
{
  // TODO There is a lot of opportunity for parallelization here
  // In general systems are run sequentially, one after the other
  //  Different Systems can run in parallel if they don't share components
  //  How to handle systems which add or remove entities?
  //  Defer creation and deletion?
  // Some systems could be run on multiple cores, such that several
  //  instances of a system each run on a subset of the entities returned
  //  in a query result.
  // Heck, entity and component data can be shared over the network to
  //  other SystemManagers to use multiple machines for one simulation

  // But this is a prototype, so here's the basic implementation
  auto systemIter = this->impl->systems.begin();
  auto queryIter = this->impl->queries.begin();
  auto systemIterEnd = this->impl->systems.end();
  for (; systemIter != systemIterEnd; ++systemIter, ++queryIter)
  {
    // Get the query results for this system
    (*systemIter)->Update(_dt, *(queryIter->Results()));
  }
}


void SystemManager::LoadSystem(System *_sys)
{
  std::shared_ptr<System> sysPtr(_sys);
  sysPtr->impl->em = this->impl->entityManager;
  sysPtr->impl->sm = this;
  EntityQuery query;
  sysPtr->Init(query);
  this->impl->entityManager->AddQuery(query);
  this->impl->systems.push_back(sysPtr);
  this->impl->queries.push_back(query);
}


}
}
