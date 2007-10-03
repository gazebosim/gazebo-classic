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
 * Desc: Factory for creating controller
 * Author: Nate Koenig
 * Date: 06 May 2007
 * SVN info: $Id$
 */

#include "GazeboError.hh"
#include "Entity.hh"
#include "gazebo.h"
#include "Controller.hh"
#include "ControllerFactory.hh"

using namespace gazebo;

void RegisterPioneer2dx_Position2d();
void RegisterSickLMS200_Laser();
void RegisterGeneric_Camera();
void RegisterFactory();
void RegisterPioneer2_Gripper();
void RegisterBandit_Actarray();

std::map<std::string, ControllerFactoryFn> ControllerFactory::controllers;

////////////////////////////////////////////////////////////////////////////////
// Register all known controllers.
void ControllerFactory::RegisterAll()
{
  RegisterPioneer2dx_Position2d();
  RegisterSickLMS200_Laser();
  RegisterGeneric_Camera();
  RegisterFactory();
  RegisterPioneer2_Gripper();
  RegisterBandit_Actarray();
}


////////////////////////////////////////////////////////////////////////////////
// Register a controller class.  Use by dynamically loaded modules
void ControllerFactory::RegisterController(std::string type, std::string classname, ControllerFactoryFn factoryfn)
{
  controllers[classname] = factoryfn;
}

////////////////////////////////////////////////////////////////////////////////
// Create a new instance of a controller.  Used by the world when reading
// the world file.
Controller *ControllerFactory::NewController(const std::string &classname, Entity *parent)
{  
  if (controllers[classname])
  {
    return (controllers[classname]) (parent);
  }
  else
  {
    std::ostringstream stream;
    stream << "Unable to make controller of type " << classname;
    gzthrow(stream.str());
  }

  return NULL;
}
