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
#ifndef GAZEBO_PHYSICS_ATMOSPHERE_HH_
#define GAZEBO_PHYSICS_ATMOSPHERE_HH_

#include <memory>
#include <string>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class World;

    /// \addtogroup gazebo_physics
    /// \{

    /// Forward declare private data class.
    class AtmospherePrivate;

    /// \class Atmosphere Atmosphere.hh physics/physics.hh
    /// \brief This models a base atmosphere class to serve as a common
    /// interface to any derived atmosphere models.
    class GZ_PHYSICS_VISIBLE Atmosphere
    {
      /// \brief Default constructor.
      /// \param[in] _world Reference to the world.
      public: explicit Atmosphere(physics::World &_world);

      /// \brief Destructor.
      public: virtual ~Atmosphere();

      /// \brief Load the atmosphere model.
      /// \param[in] _sdf Pointer to the SDF parameters.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Reset the atmosphere model.
      public: virtual void Reset();

      /// \brief Return the atmosphere model type (such as "adiabatic").
      /// \return Type of the atmosphere model.
      public: virtual std::string Type() const = 0;

      /// \brief Get a pointer to the SDF element for this atmosphere model.
      /// \return Pointer to the atmosphere SDF element.
      public: sdf::ElementPtr SDF() const;

      /// \brief Virtual callback for gztopic "~/request".
      /// \param[in] _msg Request message.
      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      /// \brief Virtual callback for gztopic "~/atmosphere".
      /// \param[in] _msg Atmosphere message.
      protected: virtual void OnAtmosphereMsg(ConstAtmospherePtr &_msg);

      /// \brief Set the sea level temperature.
      /// \param[in] _t Temperature value in kelvins.
      public: virtual void SetTemperature(const double _t);

      /// \brief Set the sea level pressure.
      /// \param[in] _pressure Pressure in pascals.
      public: virtual void SetPressure(const double _pressure);

      /// \brief Get the actual modeled temperature in kelvins at a given
      /// altitude.
      /// \param[in] _altitude Altitude above sea level in meters.
      /// \return Modeled temperature in kelvin at the specified altitude.
      public: virtual double Temperature(const double _altitude = 0.0) const;

      /// \brief Get the pressure at a specified altitude in pascals.
      /// \param[in] _altitude Altitude above sea level in meters.
      /// \return Pressure in pascals at the specified altitude.
      public: virtual double Pressure(const double _altitude = 0.0) const;

      /// \brief Get the density in kg/m^3 at a given altitude.
      /// \param[in] _altitude Altitude above sea level in meters.
      /// \return Density in kg/m^3 at the specified altitude.
      public: virtual double MassDensity(const double _altitude = 0.0) const;

      /// \brief Set the temperature gradient dT/dZ with respect to increasing
      /// altitude around sea level.
      /// \param[in] _gradient Value of the temperature gradient dT/dZ around
      /// sea level in K/m.
      public: virtual void SetTemperatureGradient(const double _gradient) = 0;

      /// \brief Get the sea level temperature gradient with respect to
      /// increasing altitude.
      /// \return Temperature gradient at sea level in K/m.
      public: double TemperatureGradient() const;

      /// \brief Update the mass density of the air at sea level using the
      /// ideal gas law.
      /// See https://en.wikipedia.org/wiki/Ideal_gas_law
      protected: void UpdateMassDensity();

      /// \brief Return the world.
      /// \return Reference to the world.
      protected: physics::World &World() const;

      /// \brief Publish response to a request.
      /// \param[in] _msg Message to be published.
      protected: void Publish(const msgs::Response &_msg) const;

      /// \brief Molar mass of the air in kg/mol
      public: static const double MOLAR_MASS;

      /// \brief Universal ideal gas constant in J/(mol.K)
      public: static const double IDEAL_GAS_CONSTANT_R;

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<AtmospherePrivate> dataPtr;
    };
    /// \}
  }
}
#endif
