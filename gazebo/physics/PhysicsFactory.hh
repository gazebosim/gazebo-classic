/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef _PHYSICSFACTORY_HH_
#define _PHYSICSFACTORY_HH_

#include <string>
#include <map>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \def PhysicsFactoryFn
    /// \brief Prototype for physics factory functions.
    typedef PhysicsEnginePtr (*PhysicsFactoryFn) (WorldPtr world);

    /// \class PhysicsFactory PhysicsFactory.hh physics/physics.hh
    /// \brief The physics factory instantiates different physics engines.
    class GZ_PHYSICS_VISIBLE PhysicsFactory
    {
      /// \brief Register everything.
      public: static void RegisterAll();

      /// \brief Register a physics class.
      /// \param[in] _className Name of the physics class.
      /// \param[in] _factoryfn Function pointer used to create a physics
      /// engine.
      public: static void RegisterPhysicsEngine(std::string _className,
                  PhysicsFactoryFn _factoryfn);

      /// \brief Create a new instance of a physics engine.
      /// \param[in] _className Name of the physics class.
      /// \param[in] _world World to pass to the created physics engine.
      public: static PhysicsEnginePtr NewPhysicsEngine(
                  const std::string &_className, WorldPtr _world);

      /// \brief Check if a physics engine is registered.
      /// \param[in] _name Name of the physics engine.
      /// \return True if physics engine is registered, false otherwise.
      public: static bool IsRegistered(const std::string &_name);

      /// \brief A list of registered physics classes.
      private: static std::map<std::string, PhysicsFactoryFn> engines;
    };

    /// \brief Static physics registration macro
    ///
    /// Use this macro to register physics engine with the server.
    /// \param[in] name Physics type name, as it appears in the world file.
    /// \param[in] classname C++ class name for the physics engine.
    #define GZ_REGISTER_PHYSICS_ENGINE(name, classname) \
    PhysicsEnginePtr New##classname(WorldPtr _world) \
    { \
      return PhysicsEnginePtr(new gazebo::physics::classname(_world)); \
    } \
    void Register##classname() \
    {\
      PhysicsFactory::RegisterPhysicsEngine(name, New##classname);\
    }
    /// \}
  }
}
#endif
