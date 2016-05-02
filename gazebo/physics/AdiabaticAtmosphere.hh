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
#ifndef GAZEBO_PHYSICS_ADIABATICATMOSPHERE_HH_
#define GAZEBO_PHYSICS_ADIABATICATMOSPHERE_HH_

#include <memory>
#include <string>

#include "gazebo/physics/Atmosphere.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class.
    class AdiabaticAtmospherePrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Adiabatic atmosphere model based on the
    /// troposphere model in the
    /// Manual of the ICAO Standard Atmosphere.
    /// http://ntrs.nasa.gov/search.jsp?R=19930083952
    /// This model assumes a specific composition of gases,
    /// the ideal gas law, hydrostatic equilibrium, and
    /// constant gradients of gravity and temperature
    /// with respect to altitude.
    /// The troposphere model is recommended for altitudes below 11 km.
    class GZ_PHYSICS_VISIBLE AdiabaticAtmosphere : public Atmosphere
    {
      /// \brief Constructor.
      /// \param[in] _world The World that uses this atmosphere model.
      public: AdiabaticAtmosphere(physics::World &_world);

      /// \brief Destructor.
      public: virtual ~AdiabaticAtmosphere();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual std::string Type() const;

      // Documentation inherited
      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      // Documentation inherited
      protected: virtual void OnAtmosphereMsg(ConstAtmospherePtr &_msg);

      // Documentation inherited
      public: virtual void SetTemperatureGradient(const double _gradient);

      // Documentation inherited
      public: virtual double Temperature(const double _altitude = 0.0) const;

      // Documentation inherited
      public: virtual double Pressure(const double _altitude = 0.0) const;

      // Documentation inherited
      public: double MassDensity(const double _altitude = 0.0) const;

      // \brief Compute the adiabatic power used internally.
      private: void ComputeAdiabaticPower();

      /// \internal
      /// \brief Private data pointer.
      protected: std::unique_ptr<AdiabaticAtmospherePrivate> dataPtr;
    };
    /// \}
  }
}
#endif
