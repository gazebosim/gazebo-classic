/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Factory for creating physics engine
 * Author: Nate Koenig 
 * Date: 21 May 2009
 * SVN info: $Id:$
 */

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
PhysicsEngine *PhysicsFactory::NewPhysicsEngine(const std::string &classname)
{
  if (engines[classname])
  {
    return (engines[classname]) ();
  }

  return NULL;
}
