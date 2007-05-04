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
 * Desc: Factory for creating models
 * Author: Andrew Howard
 * Date: 18 May 2003
 * CVS info: $Id: ModelFactory.hh,v 1.1.2.1 2006/12/16 22:41:15 natepak Exp $
 */

#ifndef MODELFACTORY_HH
#define MODELFACTORY_HH

#include <string>
#include <map>

namespace gazebo
{

// Forward declarations
class Model;


// Prototype for model factory functions
typedef Model* (*ModelFactoryFn) ();


/// @brief The model factory; the class is just for namespacing purposes.
class ModelFactory
{
  /// @brief Register all known models.
  public: static void RegisterAll();
  
  /// @brief Register a model class (called by model registration function).
  public: static void RegisterModel(std::string type, std::string  classname,
                                    ModelFactoryFn factoryfn);

  /// @brief Create a new instance of a model.  Used by the world when
  /// reading the world file.
  public: static Model *NewModel(const std::string &classname);

  // A list of registered model classes
  private: static std::map<std::string, ModelFactoryFn> models;
};


/// @brief Static model registration macro
///
/// Use this macro to register models with the server.
/// @param name Model type name, as it appears in the world file.
/// @param classname C++ class name for the model.
#define GZ_REGISTER_STATIC(name, classname) \
Model *New##classname() \
{ \
  return new classname(); \
} \
void Register##classname() \
{\
  ModelFactory::RegisterModel("static", name, New##classname);\
}


/// @brief Plugin model registration macro
///
/// Use this macro to register plugin models with the server.
/// @param name Model type name, as it appears in the world file.
/// @param classname C++ class name for the model.
#define GZ_REGISTER_PLUGIN(name, classname) \
Model *New##classname() \
{ \
  return new classname(); \
} \
extern "C" \
{ \
  int gazebo_plugin_init(void) \
  { \
    ModelFactory::RegisterModel("plugin", name, New##classname); \
    return 0;\
  }\
}

}

#endif
