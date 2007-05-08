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
 * Desc: Factory for creating ifaces
 * Author: Nate Koenig
 * Date: 06 May 2007
 * SVN info: $Id$
 */

#ifndef IFACEFACTORY_HH
#define IFACEFACTORY_HH

#include <string>
#include <map>

namespace gazebo
{

// Forward declarations
class Iface;


// Prototype for iface factory functions
typedef Iface* (*IfaceFactoryFn) ();


/// @brief The iface factory; the class is just for namespacing purposes.
class IfaceFactory
{

  /// @brief Register all known ifaces.
  public: static void RegisterAll();
  
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
}


#endif
