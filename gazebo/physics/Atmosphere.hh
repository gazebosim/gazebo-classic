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
#ifndef _ATMOSPHERE_HH_
#define _ATMOSPHERE_HH_

#include <string>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Atmosphere Atmosphere.hh physics/physics.hh
    /// \brief Base class for a atmosphere model.
    class GZ_PHYSICS_VISIBLE Atmosphere
    {
      /// \brief Default constructor.
      /// \param[in] _world Pointer to the world.
      public: explicit Atmosphere(WorldPtr _world);

      /// \brief Destructor.
      public: virtual ~Atmosphere();

      /// \brief Load the atmosphere model.
      /// \param[in] _sdf Pointer to the SDF parameters.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the atmosphere model.
      public: virtual void Init() = 0;

      /// \brief Finilize the atmosphere model.
      public: virtual void Fini();

      /// \brief Rest the atmosphere model.
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
      /// \param[in] _attr String key
      /// \sa SetParam
      /// \return The value of the parameter
      public: virtual boost::any Param(const std::string &_key) const;

      /// \brief Get a parameter from the physics model with a boolean to
      /// indicate success or failure
      /// \param[in] _key Key of the accessed param
      /// \param[out] _value Value of the accessed param
      /// \return True if the parameter was successfully retrieved
      public: virtual bool Param(const std::string &_key,
                  boost::any &_value) const;

      /// \brief Set the sea level temperature.
      /// \param[in] _t Temperature value in kelvins.
      public: virtual void SetTemperatureSL(double _t) = 0;

      /// \brief Set the sea level pressure.
      /// \param[in] _pressure Pressure in pascals.
      public: virtual void SetPressureSL(double _pressure) = 0;

      /// \brief Set the mass density of the air at sea level
      public: virtual void SetMassDensitySL(double _massDensity) = 0;

      /// \brief Get the actual modeled temperature in kelvins at a given
      /// altitude.
      /// \param[in] _altitude Altitude above sea level in meters.
      /// \return Modeled temperature in kelvin at the specified
      /// altitude.
      public: virtual double Temperature(double _altitude) const = 0;

      /// \brief Get the actual modeled sea level temperature in kelvins.
      /// \return Modeled temperature in kelvin at sea level.
      public: virtual double TemperatureSL() const
              { return Temperature(0.0); }

      /// \brief Get the pressure at a specified altitude in pascals.
      /// \param[in] _altitude Altitude above sea level in meters.
      /// \return Pressure in pascals at the specified altitude.
      public: virtual double Pressure(double _altitude) const = 0;

      /// \brief Get the sea level pressure in pascals.
      /// \return Pressure in pascals at sea level.
      public: virtual double PressureSL() const { return Pressure(0.0); }

      /// \brief Get the density in kg/m^3 at a given altitude in meters.
      /// \param[in] _altitude Altitude above sea level in meters.
      /// \return Density in kg/m^3 at the specified altitude.
      public: virtual double MassDensity(double _altitude) const = 0;

      /// \brief Get the sea level density in kg/m^3
      /// \return Density in kg/m^3 at sea level.
      public: virtual double MassDensitySL(void) const
              { return MassDensity(0.0); }

      /// \brief Set the temperature gradient dT/dZ around sea level
      /// \param[in] _gradient Value of the temperature gradient dT/dZ around
      /// sea level [K/m]
      public: virtual void SetTemperatureGradientSL(double _gradient) = 0;

      /// \brief Get the sea level temperature gradient.
      /// \return Temperature gradient at sea level.
      public: virtual double TemperatureGradientSL() const
              { return this->temperatureGradientSL; }

      /// \brief Pointer to the world.
      protected: WorldPtr world;

      /// \brief Our SDF values.
      protected: sdf::ElementPtr sdf;

      /// \brief Node for communication.
      protected: transport::NodePtr node;

      /// \brief Response publisher.
      protected: transport::PublisherPtr responsePub;

      /// \brief Subscribe to the atmosphere topic.
      protected: transport::SubscriberPtr atmosphereSub;

      /// \brief Subscribe to the request topic.
      protected: transport::SubscriberPtr requestSub;

      /// \brief Temperature at sea level in kelvins.
      protected: double temperatureSL;

      /// \brief Temperature gradient at sea level in K/m.
      protected: double temperatureGradientSL;

      /// \brief Pressure of the air at sea level in pascals.
      protected: double pressureSL;

      /// \brief Mass density of the air at sea level in kg/m^3.
      protected: double massDensitySL;
    };
    /// \}
  }
}
#endif
