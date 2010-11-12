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
 * SVN info:$
 */

#ifndef PHYSICSFACTORY_HH
#define PHYSICSFACTORY_HH

#include <string>
#include <map>

namespace gazebo
{
  
  // Forward declarations
  class PhysicsEngine;
  class World;
  
  // Prototype for sensor factory functions
  typedef PhysicsEngine* (*PhysicsFactoryFn) (World *world);
  
  /// \addtogroup gazebo_physics
  /// \brief The physics factory
  /// \{
  
  /// \brief The physics factory
  class PhysicsFactory
  {
    /// \brief Register everything
    public: static void RegisterAll();

    /// \brief Register a physics class.
    public: static void RegisterPhysicsEngine(std::string classname,
                                        PhysicsFactoryFn factoryfn);
  
    /// \brief Create a new instance of a physics engine.  
    public: static PhysicsEngine *NewPhysicsEngine(const std::string &classname, World *world);
  
    /// \brief A list of registered physics classes
    private: static std::map<std::string, PhysicsFactoryFn> engines;
  };
  
  
  /// \brief Static sensor registration macro
  ///
  /// Use this macro to register sensors with the server.
  /// @param name Physics type name, as it appears in the world file.
  /// @param classname C++ class name for the sensor.
#define GZ_REGISTER_PHYSICS_ENGINE(name, classname) \
PhysicsEngine *New##classname(World *world) \
{ \
  return new classname(world); \
} \
void Register##classname() \
{\
  PhysicsFactory::RegisterPhysicsEngine(name, New##classname);\
}
//StaticPluginRegister Registered##classname (Register##classname);
  
  /// \}
}

#endif
