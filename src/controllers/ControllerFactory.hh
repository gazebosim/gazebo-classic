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
 * Desc: Factory for creating controllers
 * Author: Nate Koenig
 * Date: 06 May 2007
 * SVN info: $Id$
 */

#ifndef CONTROLLERFACTORY_HH
#define CONTROLLERFACTORY_HH

#include <string>
#include <map>
#include "common/StaticPluginRegister.hh"

namespace gazebo
{

/// \addtogroup gazebo_controller
/// \brief Factory for creating controllers
/// \{

// Forward declarations
class Controller;
class Entity;


/// Prototype for controller factory functions
typedef Controller* (*ControllerFactoryFn) (Entity *parent);


/// \brief The controller factory; the class is just for namespacing purposes.
class ControllerFactory
{
  /// \brief Register a controller class 
  /// (called by controller registration function).
  public: static void RegisterController(std::string type, std::string classname, ControllerFactoryFn factoryfn);

  /// \brief Create a new instance of a controller.  Used by the world when
  /// reading the world file.
  public: static Controller *NewController(const std::string &classname, Entity *parent);
  
  /// \brief Load a controller plugin. Used by Model and Sensor when creating controllers.
  public: static void LoadPlugin(const std::string &plugin, const std::string &classname);

  // A list of registered controller classes
  private: static std::map<std::string, ControllerFactoryFn> controllers;

};


/// \brief Static controller registration macro
///
/// Use this macro to register controllers with the server.
/// \param name Controller type name, as it appears in the world file.
/// \param classname C++ class name for the controller.
#define GZ_REGISTER_STATIC_CONTROLLER(name, classname) \
Controller *New##classname(Entity *entity) \
{ \
  return new classname(entity); \
} \
void Register##classname() \
{\
  ControllerFactory::RegisterController("static", name, New##classname);\
}\
StaticPluginRegister Registered##classname (Register##classname);

/// \brief Dynamic controller registration macro
///
/// Use this macro to register plugin controllers with the server.
/// \param name Controller type name, as it appears in the world file.
/// \param classname C++ class name for the controller.
#define GZ_REGISTER_DYNAMIC_CONTROLLER(name, classname) \
Controller *New##classname(Entity *entity) \
{ \
  return new classname(entity); \
} \
extern "C" void RegisterPluginController(); \
void RegisterPluginController() \
{\
  ControllerFactory::RegisterController("dynamic", name, New##classname);\
}

/// \}
}

#endif
