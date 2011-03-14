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
 * Desc: Factory for creating controller
 * Author: Nate Koenig
 * Date: 06 May 2007
 * SVN info: $Id$
 */

#include "gazebo_config.h"

#include "common/GazeboError.hh"
#include "Entity.hh"
#include "gz.h"
#include "Controller.hh"
#include "ControllerFactory.hh"

#include <sys/stat.h>
#include "Simulator.hh"
#include "common/GazeboConfig.hh"
#include "common/GazeboMessage.hh"

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
    gzmsg(0) << "Error Loading Controller of type [" <<  classname << "], skipping\n" << std::endl;
    return NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load a controller plugin. Used by Model and Sensor when creating controllers.
void ControllerFactory::LoadPlugin(const std::string &plugin, const std::string &classname)
{
  // search and expand plugin with full path by searching GazeboConfig::pluginPaths,
  // otherwise, leave as is, and let LD_LIBRARY_PATH take care of business
  struct stat st;
  bool found = false;
  std::string fullname;
  std::list<std::string>::iterator iter;
  std::list<std::string> pluginPaths=Simulator::Instance()->GetGazeboConfig()->GetPluginPaths();
  for (iter=pluginPaths.begin(); iter!=pluginPaths.end(); ++iter)
  {
    fullname = (*iter)+std::string("/")+plugin;
    if (stat(fullname.c_str(), &st) == 0) {found=true; break;}
  }
  if (!found) fullname = plugin;

#ifdef HAVE_DL

  void* handle = dlopen(fullname.c_str(), RTLD_LAZY|RTLD_GLOBAL);
  if (!handle)
  {
    std::ostringstream stream;
    stream << "Failed to load " << fullname << ": " << dlerror();
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
	
	lt_dlhandle handle = lt_dlopenext(fullname.c_str());
	
	if (!handle)
	{
		std::ostringstream stream;
		stream << "Failed to load " << fullname << ": " << lt_dlerror();
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
