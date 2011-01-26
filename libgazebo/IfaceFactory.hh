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
 * Desc: Factory for creating ifaces
 * Author: Nate Koenig
 * Date: 06 May 2007
 * SVN info: $Id$
 */

#ifndef IFACEFACTORY_HH
#define IFACEFACTORY_HH

#include <string>
#include <map>

struct LibgazeboPluginRegister
{
  LibgazeboPluginRegister(void (*initFunc)())
  {
    (*initFunc)();
  }
};

namespace libgazebo
{
  class Iface;

  // Prototype for iface factory functions
  typedef Iface* (*IfaceFactoryFn) ();


  /// @brief The iface factory; the class is just for namespacing purposes.
  class IfaceFactory
  {
  
    /// @brief Register all known ifaces.
    //public: static void RegisterAll();
    
    /// @brief Register a iface class 
    /// (called by iface registration function).
    public: static void RegisterIface(std::string classname, IfaceFactoryFn factoryfn);
  
    /// @brief Create a new instance of a iface.  Used by the world when
    /// reading the world file.
    public: static Iface *NewIface(const std::string &classname);
  
    // A list of registered iface classes
    private: static std::map<std::string, IfaceFactoryFn> ifaces;
  
  };

}

/// @brief Static iface registration macro
///
/// Use this macro to register ifaces with the server.
/// @param name Iface type name, as it appears in the world file.
/// @param classname C++ class name for the iface.
#define GZ_REGISTER_IFACE(name, classname) \
Iface *New##classname() \
{ \
  return new classname(); \
} \
void Register##classname() \
{\
  IfaceFactory::RegisterIface(name, New##classname);\
}\
LibgazeboPluginRegister Registered##classname (Register##classname);

#endif
