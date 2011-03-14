/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/*
 * Desc: Factory for creating physics engine
 * Author: Nate Koenig 
 * Date: 21 May 2009
 * SVN info: $Id:$
 */

#include "World.hh"
#include "PhysicsEngine.hh"
#include "PhysicsFactory.hh"
#include "gazebo_config.h"

#ifdef INCLUDE_ODE
void RegisterODEPhysics();
#endif

#ifdef INCLUDE_BULLET
void RegisterBulletPhysics();
#endif

using namespace gazebo;
using namespace physics;


std::map<std::string, PhysicsFactoryFn> PhysicsFactory::engines;

////////////////////////////////////////////////////////////////////////////////
/// Register everything
void PhysicsFactory::RegisterAll()
{
#ifdef INCLUDE_ODE
  RegisterODEPhysics();
#endif

#ifdef INCLUDE_BULLET
  RegisterBulletPhysics();
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Register a physics engine class.
void PhysicsFactory::RegisterPhysicsEngine(std::string classname,
                                     PhysicsFactoryFn factoryfn)
{
  engines[classname] = factoryfn;
}

////////////////////////////////////////////////////////////////////////////////
// Create a new instance of a physics engine.
PhysicsEngine *PhysicsFactory::NewPhysicsEngine(const std::string &classname, World *world)
{
  if (engines[classname])
  {
    return (engines[classname]) (world);
  }

  return NULL;
}
