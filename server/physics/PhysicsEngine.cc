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
/* Desc: The base class for all physics engines
 * Author: Nate Koenig
 * Date: 11 June 2007
 * SVN: $Id$
 */

#include <boost/thread/recursive_mutex.hpp>

#include "Shape.hh"
#include "PhysicsEngine.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
PhysicsEngine::PhysicsEngine()
{
  Param::Begin(&this->parameters);
  this->gravityP = new ParamT<Vector3>("gravity",Vector3(0.0, -9.80665, 0.0), 0);
  this->updateRateP = new ParamT<double>("updateRate", 0.0, 0);
  this->stepTimeP = new ParamT<double>("stepTime",0.025,0);
  Param::End();

  this->mutex = new boost::recursive_mutex();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
PhysicsEngine::~PhysicsEngine()
{
  delete this->gravityP;
  delete this->updateRateP;
  delete this->stepTimeP;

  delete this->mutex;
  this->mutex = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the gavity vector
Vector3 PhysicsEngine::GetGravity() const
{
  return this->gravityP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the time between each update cycle
double PhysicsEngine::GetUpdateRate() const
{
  return this->updateRateP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the time between each update cycle
double PhysicsEngine::GetStepTime() const
{
  return this->stepTimeP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Lock the physics engine mutex
void PhysicsEngine::LockMutex()
{
  this->mutex->lock();
}

////////////////////////////////////////////////////////////////////////////////
/// Lock the physics engine mutex
void PhysicsEngine::UnlockMutex()
{
  this->mutex->unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a geom
Geom *PhysicsEngine::CreateGeom(std::string typeName, Body *body)
{
  for (unsigned int i = 0; i < Shape::TYPE_COUNT; i++)
    if (typeName == Shape::TypeNames[i])
      return this->CreateGeom( (Shape::Type)i, body );

  return NULL; 
}
