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
#ifndef _ADIABATICATMOSPHERE_HH_
#define _ADIABATICATMOSPHERE_HH_

#include <string>

#include "gazebo/physics/Atmosphere.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class.
    class AdiabaticAtmospherePrivate;

    /// \brief Adiabatic atmosphere model.
    /// Models the adiabatic atmosphere.
    class GZ_PHYSICS_VISIBLE AdiabaticAtmosphere : public Atmosphere
    {
      /// \brief Constructor.
      /// \param[in] _world The World that uses this atmosphere model.
      public: AdiabaticAtmosphere(WorldPtr _world);

      /// \brief Destructor.
      public: virtual ~AdiabaticAtmosphere();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual std::string Type() const { return "adiabatic"; }

      // Documentation inherited
      public: virtual bool SetParam(const std::string &_key,
                            const boost::any &_value);

      // Documentation inherited
      public: virtual boost::any Param(const std::string &_key) const;

      // Documentation inherited
      public: virtual bool Param(const std::string &_key,
                  boost::any &_value) const;

      // Documentation inherited
      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      // Documentation inherited
      protected: virtual void OnAtmosphereMsg(ConstAtmospherePtr &_msg);

      // Documentation inherited
      public: virtual void SetTemperatureSL(double _t);

      // Documentation inherited
      public: virtual void SetTemperatureGradientSL(double _gradient);

      // Documentation inherited
      public: virtual void SetPressureSL(double _gradient);

      // Documentation inherited
      public: virtual void SetMassDensitySL(double _massDensity);

      // Documentation inherited
      public: virtual double Temperature(double _altitude) const;

      // Documentation inherited
      virtual double Pressure(double _altitude) const;

      // Documentation inherited
      public: double MassDensity(double _altitude) const;

      /// \brief Molar mass of the air in kg/mol
      public: static const double MOLAR_MASS;

      /// \brief Universal ideal gas constant in J/(mol.K)
      public: static const double IDEAL_GAS_CONSTANT_R;

      /// \internal
      /// \brief Private data pointer.
      private: AdiabaticAtmospherePrivate *dataPtr;
    };
  }
}
#endif
