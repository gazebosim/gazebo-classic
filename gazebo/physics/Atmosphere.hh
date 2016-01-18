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
#ifndef _GAZEBO_PHYSICS_ATMOSPHERE_HH_
#define _GAZEBO_PHYSICS_ATMOSPHERE_HH_

#include <boost/any.hpp>
#include <string>
#include <memory>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// Forward declare private data class.
    class AtmospherePrivate;

    /// \class Atmosphere Atmosphere.hh physics/physics.hh
    /// \brief Base class for a atmosphere model.
    class GZ_PHYSICS_VISIBLE Atmosphere
    {
      /// \brief Default constructor.
      /// \param[in] _world Pointer to the world.
      /// \param[in] _dataPtr Pointer to private data.
      public: explicit Atmosphere(WorldPtr _world, AtmospherePrivate &_dataPtr);

      /// \brief Destructor.
      public: virtual ~Atmosphere();

      /// \brief Load the atmosphere model.
      /// \param[in] _sdf Pointer to the SDF parameters.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the atmosphere model.
      public: virtual void Init() = 0;

      /// \brief Finilize the atmosphere model.
      public: virtual void Fini();

      /// \brief Reset the atmosphere model.
      public: virtual void Reset() {}

      /// \brief Return the atmosphere model type (adiabatic).
      /// \return Type of the atmosphere model.
      public: virtual std::string Type() const = 0;

      /// \brief Get a pointer to the SDF element for this atmosphere model.
      /// \return Pointer to the atmosphere SDF element.
      public: sdf::ElementPtr SDF() const;

      /// \brief virtual callback for gztopic "~/request".
      /// \param[in] _msg Request message.
      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      /// \brief virtual callback for gztopic "~/atmosphere".
      /// \param[in] _msg Atmosphere message.
      protected: virtual void OnAtmosphereMsg(ConstAtmospherePtr &_msg);

      /// \brief Set a parameter of the atmosphere model.
      /// See SetParam documentation for descriptions of duplicate parameters.
      /// \param[in] _key String key
      /// \param[in] _value The value to set to
      /// \return true if SetParam is successful, false if operation fails.
      public: virtual bool SetParam(const std::string &_key,
                  const boost::any &_value);

      /// \brief Get an parameter of the physics model
      /// \param[in] _key String key
      /// \return The value of the parameter
      /// \sa SetParam
      public: virtual boost::any Param(const std::string &_key) const;

      /// \brief Get a parameter from the physics model with a boolean to
      /// indicate success or failure
      /// \param[in] _key Key of the accessed param
      /// \param[out] _value Value of the accessed param
      /// \return True if the parameter was successfully retrieved
      /// \sa SetParam
      public: virtual bool Param(const std::string &_key,
                  boost::any &_value) const;

      /// \brief Set the sea level temperature.
      /// \param[in] _t Temperature value in kelvins.
      public: virtual void SetTemperatureSL(const double _t) = 0;

      /// \brief Set the sea level pressure.
      /// \param[in] _pressure Pressure in pascals.
      public: virtual void SetPressureSL(const double _pressure) = 0;

      /// \brief Set the mass density of the air at sea level
      /// \param[in] _massDensity Mass density of the air in kg/m^3.
      public: virtual void SetMassDensitySL(const double _massDensity) = 0;

      /// \brief Get the actual modeled temperature in kelvins at a given
      /// altitude.
      /// \param[in] _altitude Altitude above sea level in meters.
      /// \return Modeled temperature in kelvin at the specified
      /// altitude.
      public: virtual double Temperature(const double _altitude) const = 0;

      /// \brief Get the actual modeled sea level temperature in kelvins.
      /// \return Modeled temperature in kelvin at sea level.
      public: virtual double TemperatureSL() const;

      /// \brief Get the pressure at a specified altitude in pascals.
      /// \param[in] _altitude Altitude above sea level in meters.
      /// \return Pressure in pascals at the specified altitude.
      public: virtual double Pressure(const double _altitude) const = 0;

      /// \brief Get the sea level pressure in pascals.
      /// \return Pressure in pascals at sea level.
      public: virtual double PressureSL() const;

      /// \brief Get the density in kg/m^3 at a given altitude in meters.
      /// \param[in] _altitude Altitude above sea level in meters.
      /// \return Density in kg/m^3 at the specified altitude.
      public: virtual double MassDensity(const double _altitude) const = 0;

      /// \brief Get the sea level density in kg/m^3
      /// \return Density in kg/m^3 at sea level.
      public: virtual double MassDensitySL() const;

      /// \brief Set the temperature gradient dT/dZ around sea level
      /// \param[in] _gradient Value of the temperature gradient dT/dZ around
      /// sea level [K/m]
      public: virtual void SetTemperatureGradientSL(const double _gradient) = 0;

      /// \brief Get the sea level temperature gradient.
      /// \return Temperature gradient at sea level.
      public: virtual double TemperatureGradientSL() const;

      /// \internal
      /// \brief Private data pointer.
      protected: std::unique_ptr<AtmospherePrivate> dataPtr;
    };
    /// \}
  }
}
#endif
