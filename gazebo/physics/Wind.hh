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
#ifndef GAZEBO_PHYSICS_WIND_HH_
#define GAZEBO_PHYSICS_WIND_HH_

#include <string>
#include <functional>
#include <memory>
#include <boost/any.hpp>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    // Forward declare private data class.
    class WindPrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class Wind Wind.hh physics/physics.hh
    /// \brief Base class for wind.
    class GZ_PHYSICS_VISIBLE Wind
    {
      /// \brief Default constructor.
      /// \param[in] _world Reference to the world.
      /// \param[in] _sdf SDF element parameters for the wind.
      public: explicit Wind(World &_world, sdf::ElementPtr _sdf);

      /// \brief Destructor.
      public: virtual ~Wind();

      /// \brief Load the wind.
      /// \param[in] _sdf Pointer to the SDF parameters.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Set a parameter of the wind.
      /// See SetParam documentation for descriptions of duplicate parameters.
      /// \param[in] _key String key
      /// Below is a list of _key parameter definitions:
      ///       -# "linear_vel" (Vector3d) - wind linear velocity
      ///
      /// \param[in] _value The value to set to
      /// \return true if SetParam is successful, false if operation fails.
      public: bool SetParam(const std::string &_key,
                            const boost::any &_value);

      /// \brief Get a wind parameter
      /// \param[in] _attr String key
      /// \sa SetParam
      /// \return The value of the parameter
      public: boost::any Param(const std::string &_key) const;

      /// \brief Get a wind parameter with a boolean to
      /// indicate success or failure
      /// \param[in] _key Key of the accessed param
      /// \param[out] _value Value of the accessed param
      /// \return True if the parameter was successfully retrieved
      public: bool Param(const std::string &_key, boost::any &_value) const;

      /// \brief virtual callback for gztopic "~/wind".
      /// \param[in] _msg Wind message.
      private: void OnWindMsg(ConstWindPtr &_msg);

      /// \brief virtual callback for gztopic "~/request".
      /// \param[in] _msg Request message.
      private: void OnRequest(ConstRequestPtr &_msg);

      /// \brief Get the wind velocity at an entity location in the
      /// world coordinate frame.
      /// \param[in] _entity Entity at which location the wind is applied.
      /// \return Linear velocity of the wind.
      public: ignition::math::Vector3d WorldLinearVel(const Entity *_entity)
          const;

      /// \brief Get the wind velocity at an entity location.
      /// \param[in] _entity Entity at which location the wind is applied.
      /// \return Linear velocity of the wind.
      public: ignition::math::Vector3d RelativeLinearVel(const Entity *_entity)
          const;

      /// \brief Get the global wind velocity.
      /// \return Linear velocity of the wind.
      public: const ignition::math::Vector3d& LinearVel(void) const;

      /// \brief Set the global wind velocity.
      /// \param[in] _vel Global wind velocity.
      public: void SetLinearVel(const ignition::math::Vector3d& _vel);

      /// \brief Setup function to compute the wind.
      /// \param[in] _linearVelFunc The function callback that is used
      /// to calculate the wind's velocity. The parameters to the
      /// function callback is a reference to an instance of
      /// Wind and a pointer to an entity in the scene. The function must
      /// return the new wind velocity as a vector.
      public: void SetLinearVelFunc(std::function< ignition::math::Vector3d (
          const Wind *_wind, const Entity *_entity) > _linearVelFunc);

      /// \brief Get the global wind velocity, ignoring the entity.
      /// \param[in] _wind Reference to the wind.
      /// \param[in] _entity Pointer to an entity at which location the wind
      /// velocity is to be calculated.
      /// \return Wind's velocity at entity's location.
      private: ignition::math::Vector3d LinearVelDefault(const Wind *_wind,
          const Entity *_entity);

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<WindPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
