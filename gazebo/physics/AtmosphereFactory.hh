/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _ATMOSPHEREFACTORY_HH_
#define _ATMOSPHEREFACTORY_HH_

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

    /// \def AtmosphereFactoryFn
    /// \brief Prototype for atmosphere factory functions.
    typedef AtmospherePtr (*AtmosphereFactoryFn) (WorldPtr world);

    /// \class AtmosphereFactory AtmosphereFactory.hh physics/physics.hh
    /// \brief The atmosphere factory instantiates different atmosphere models.
    class GZ_PHYSICS_VISIBLE AtmosphereFactory
    {
      /// \brief Register everything.
      public: static void RegisterAll();

      /// \brief Register an atmosphere class.
      /// \param[in] _className Name of the atmosphere class.
      /// \param[in] _factoryfn Function pointer used to create an atmosphere
      /// model.
      public: static void RegisterAtmosphere(std::string _className,
                  AtmosphereFactoryFn _factoryfn);

      /// \brief Create a new instance of an atmosphere model.
      /// \param[in] _className Name of the atmosphere class.
      /// \param[in] _world World to pass to the created atmosphere model.
      public: static AtmospherePtr NewAtmosphere(
                  const std::string &_className, WorldPtr _world);

      /// \brief Check if an atmosphere model is registered.
      /// \param[in] _name Name of the atmosphere model.
      /// \return True if atmosphere model is registered, false otherwise.
      public: static bool IsRegistered(const std::string &_name);

      /// \brief A list of registered atmosphere classes.
      private: static std::map<std::string, AtmosphereFactoryFn> models;
    };

    /// \brief Static atmosphere registration macro
    ///
    /// Use this macro to register atmosphere model with the server.
    /// \param[in] name Atmosphere type name, as it appears in the world file.
    /// \param[in] classname C++ class name for the atmosphere model.
    #define GZ_REGISTER_ATMOSPHERE_MODEL(name, classname) \
    GZ_PHYSICS_VISIBLE AtmospherePtr New##classname(WorldPtr _world) \
    { \
      return AtmospherePtr(new gazebo::physics::classname(_world)); \
    } \
    GZ_PHYSICS_VISIBLE \
    void Register##classname() \
    {\
      AtmosphereFactory::RegisterAtmosphere(name, New##classname);\
    }
    /// \}
  }
}
#endif
