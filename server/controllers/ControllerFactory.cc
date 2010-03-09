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

#include "config.h"

#include "GazeboError.hh"
#include "Entity.hh"
#include "gz.h"
#include "Controller.hh"
#include "ControllerFactory.hh"

#ifdef HAVE_DL
#include <dlfcn.h>
#elif HAVE_LTDL
#include <ltdl.h>
#endif

using namespace gazebo;

std::map<std::string, ControllerFactoryFn> ControllerFactory::controllers;

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

////////////////////////////////////////////////////////////////////////////////
// Load a controller plugin. Used by Model and Sensor when creating controllers.
void ControllerFactory::LoadPlugin(const std::string &plugin, const std::string &classname)
{
#ifdef HAVE_DL

  std::cerr << "\n\n\n\n\USING DL\n\n\n\n";
  void* handle = dlopen(plugin.c_str(), RTLD_LAZY|RTLD_GLOBAL);
  if (!handle)
  {
    std::ostringstream stream;
    stream << "Failed to load " << plugin << ": " << dlerror();
    gzthrow(stream.str());
  }

  std::string registerName = "RegisterPluginController";
  void *(*registerFunc)() = (void *(*)())dlsym(handle, registerName.c_str());
  if(!registerFunc)
  {
    std::ostringstream stream;
    stream << "Failed to resolve " << registerName << ": " << dlerror();
    gzthrow(stream.str());
  }

	// Register the new controller.
  registerFunc();

#elif HAVE_LTDL

	static bool init_done = false;

	if (!init_done)
	{
		int errors = lt_dlinit();
		if (errors)
		{
			std::ostringstream stream;
		    stream << "Error(s) initializing dynamic loader (" 
               << errors << ", " << lt_dlerror() << ")";
		    gzthrow(stream.str());
		}
		else
			init_done = true;
	}
	
	lt_dlhandle handle = lt_dlopenext(plugin.c_str());
	
	if (!handle)
	{
		std::ostringstream stream;
		stream << "Failed to load " << plugin << ": " << lt_dlerror();
		gzthrow(stream.str());
	}
	
	std::string registerName = "RegisterPluginController";
	void *(*registerFunc)() = (void *(*)())lt_dlsym(handle, registerName.c_str());
	if(!registerFunc)
	{
    	std::ostringstream stream;
    	stream << "Failed to resolve " << registerName << ": " << lt_dlerror();
    	gzthrow(stream.str());
	}

	// Register the new controller.
  registerFunc();

#else // HAVE_LTDL
	
    gzthrow("Cannot load plugins as libtool is not installed.");
	
#endif // HAVE_LTDL

}
