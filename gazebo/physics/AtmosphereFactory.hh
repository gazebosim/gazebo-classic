/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_ATMOSPHEREFACTORY_HH_
#define GAZEBO_PHYSICS_ATMOSPHEREFACTORY_HH_

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
    typedef std::unique_ptr<Atmosphere> (*AtmosphereFactoryFn) (World &world);

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
      public: static void RegisterAtmosphere(const std::string &_className,
                  AtmosphereFactoryFn _factoryfn);

      /// \brief Create a new instance of an atmosphere model.
      /// \param[in] _className Name of the atmosphere class.
      /// \param[in] _world World to pass to the created atmosphere model.
      /// \return Unique pointer to the atmosphere model.
      public: static std::unique_ptr<Atmosphere> NewAtmosphere(
                  const std::string &_className, World &_world);

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
    /// \param[in] _name Atmosphere type name, as it appears in the world file.
    /// \param[in] _classname C++ class name for the atmosphere model.
    #define GZ_REGISTER_ATMOSPHERE_MODEL(_name, _classname) \
    GZ_PHYSICS_VISIBLE std::unique_ptr<Atmosphere> \
        New##_classname(World &_world) \
    { \
      return std::unique_ptr<Atmosphere>( \
          new gazebo::physics::_classname(_world)); \
    } \
    GZ_PHYSICS_VISIBLE \
    void Register##_classname() \
    {\
      AtmosphereFactory::RegisterAtmosphere(_name, New##_classname);\
    }
    /// \}
  }
}
#endif
