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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <sdf/sdf.hh>

#include "gazebo/util/Diagnostics.hh"
#include "gazebo/common/Assert.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/AtmosphereFactory.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/physics/AdiabaticAtmosphere.hh"
#include "gazebo/physics/AdiabaticAtmospherePrivate.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_ATMOSPHERE_MODEL("adiabatic", AdiabaticAtmosphere)

const double AdiabaticAtmosphere::MOLAR_MASS = 0.0289644;
const double AdiabaticAtmosphere::IDEAL_GAS_CONSTANT_R = 8.3144621;

//////////////////////////////////////////////////
AdiabaticAtmosphere::AdiabaticAtmosphere(WorldPtr _world)
  : Atmosphere(_world)
  , dataPtr(new AdiabaticAtmospherePrivate)
{
}

//////////////////////////////////////////////////
AdiabaticAtmosphere::~AdiabaticAtmosphere()
{
}

//////////////////////////////////////////////////
void AdiabaticAtmosphere::Load(sdf::ElementPtr _sdf)
{
  Atmosphere::Load(_sdf);
}

//////////////////////////////////////////////////
void AdiabaticAtmosphere::Init(void)
{
  AdiabaticAtmospherePrivate *dPtr =
      reinterpret_cast<AdiabaticAtmospherePrivate *>(this->dataPtr.get());

  dPtr->adiabaticPower = MOLAR_MASS *
      -this->World()->GetPhysicsEngine()->GetGravity().z /
      (this->TemperatureGradientSL() * IDEAL_GAS_CONSTANT_R);
}

//////////////////////////////////////////////////
std::string AdiabaticAtmosphere::Type() const
{
  return "adiabatic";
}

/////////////////////////////////////////////////
void AdiabaticAtmosphere::OnRequest(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "atmosphere_info")
  {
    msgs::Atmosphere atmosphereMsg;
    atmosphereMsg.set_type(msgs::Atmosphere::ADIABATIC);
    atmosphereMsg.set_enable_atmosphere(
            this->World()->EnableAtmosphere());
    atmosphereMsg.set_temperature(this->TemperatureSL());
    atmosphereMsg.set_pressure(this->PressureSL());
    atmosphereMsg.set_mass_density(this->MassDensitySL());

    response.set_type(atmosphereMsg.GetTypeName());
    atmosphereMsg.SerializeToString(serializedData);
    this->Publish(response);
  }
}

/////////////////////////////////////////////////
void AdiabaticAtmosphere::OnAtmosphereMsg(ConstAtmospherePtr &_msg)
{
  // Parent class handles many generic parameters
  // This should be done first so that the profile settings
  // can be over-ridden by other message parameters.
  Atmosphere::OnAtmosphereMsg(_msg);

  if (_msg->has_enable_atmosphere())
    this->World()->EnableAtmosphere(_msg->enable_atmosphere());

  if (_msg->has_temperature())
    this->SetTemperatureSL(_msg->temperature());

  if (_msg->has_pressure())
    this->SetPressureSL(_msg->pressure());

  if (_msg->has_mass_density())
    this->SetMassDensitySL(_msg->mass_density());
}

//////////////////////////////////////////////////
bool AdiabaticAtmosphere::SetParam(const std::string &_key,
                                   const boost::any &_value)
{
  return Atmosphere::SetParam(_key, _value);
}

//////////////////////////////////////////////////
boost::any AdiabaticAtmosphere::Param(const std::string &_key) const
{
  boost::any value;
  this->Param(_key, value);
  return value;
}

//////////////////////////////////////////////////
bool AdiabaticAtmosphere::Param(const std::string &_key,
                                   boost::any &_value) const
{
  return Atmosphere::Param(_key, _value);
}

//////////////////////////////////////////////////
void AdiabaticAtmosphere::SetTemperatureSL(const double _temperature)
{
  Atmosphere::SetTemperatureSL(_temperature);
}

//////////////////////////////////////////////////
void AdiabaticAtmosphere::SetTemperatureGradientSL(const double _gradient)
{
  Atmosphere::SetTemperatureGradientSL(_gradient);
  this->Init();
}

//////////////////////////////////////////////////
void AdiabaticAtmosphere::SetPressureSL(const double _pressure)
{
  Atmosphere::SetPressureSL(_pressure);
}

//////////////////////////////////////////////////
void AdiabaticAtmosphere::SetMassDensitySL(const double _massDensity)
{
  Atmosphere::SetMassDensitySL(_massDensity);
}

//////////////////////////////////////////////////
double AdiabaticAtmosphere::Temperature(const double _altitude) const
{
  double temperature = this->TemperatureSL() +
      this->TemperatureGradientSL() * _altitude;
  return temperature;
}

//////////////////////////////////////////////////
double AdiabaticAtmosphere::Pressure(const double _altitude) const
{
  double pressure = this->PressureSL() *
      pow(1 - (this->TemperatureGradientSL() * _altitude) /
          this->TemperatureSL(), this->dataPtr->adiabaticPower);
  return pressure;
}

//////////////////////////////////////////////////
double AdiabaticAtmosphere::MassDensity(const double _altitude) const
{
  double massDensity = this->MassDensitySL() *
      pow(1 + (this->TemperatureGradientSL() * _altitude) /
          this->TemperatureSL(), this->dataPtr->adiabaticPower - 1);
  return massDensity;
}
