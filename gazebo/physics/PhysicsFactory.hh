/*
 * Copyright 2011 Nate Koenig
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
 * Desc: Factory for creating physics engine
 * Author: Nate Koenig
 * Date: 21 May 2009
 */

#ifndef PHYSICSFACTORY_HH
#define PHYSICSFACTORY_HH

#include <string>
#include <map>

#include "physics/PhysicsTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    // Prototype for physics factory functions
    typedef PhysicsEnginePtr (*PhysicsFactoryFn) (WorldPtr world);

    /// \brief The physics factory
    class PhysicsFactory
    {
      /// \brief Register everything
      public: static void RegisterAll();

      /// \brief Register a physics class.
      public: static void RegisterPhysicsEngine(std::string classname,
                  PhysicsFactoryFn factoryfn);

      /// \brief Create a new instance of a physics engine.
      public: static PhysicsEnginePtr NewPhysicsEngine(
                  const std::string &classname, WorldPtr world);

      /// \brief A list of registered physics classes
      private: static std::map<std::string, PhysicsFactoryFn> engines;
    };


    /// \brief Static physics registration macro
    ///
    /// Use this macro to register physics engine with the server.
    /// @param name Physics type name, as it appears in the world file.
    /// @param classname C++ class name for the physics engine.
#define GZ_REGISTER_PHYSICS_ENGINE(name, classname) \
    PhysicsEnginePtr New##classname(WorldPtr world) \
    { \
      return PhysicsEnginePtr(new gazebo::physics::classname(world)); \
    } \
    void Register##classname() \
    {\
      PhysicsFactory::RegisterPhysicsEngine(name, New##classname);\
    }
    /// \}
  }
}
#endif
