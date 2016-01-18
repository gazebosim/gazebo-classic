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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Atmosphere.hh"
#include "gazebo/physics/PresetManager.hh"
#include "gazebo/physics/AtmospherePrivate.hh"

using namespace gazebo;
using namespace physics;

/// This models a base atmosphere class to serve as a common
/// interface to any derived atmosphere models.

//////////////////////////////////////////////////
Atmosphere::Atmosphere(WorldPtr _world, AtmospherePrivate &_dataPtr)
  : dataPtr(&_dataPtr)
{
  this->dataPtr->world = _world;
  this->dataPtr->temperatureSL = 0;
  this->dataPtr->temperatureGradientSL = 0;
  this->dataPtr->pressureSL = 0;
  this->dataPtr->massDensitySL = 0;

  this->dataPtr->sdf.reset(new sdf::Element);
  sdf::initFile("atmosphere.sdf", this->dataPtr->sdf);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->world->GetName());
  this->dataPtr->atmosphereSub = this->dataPtr->node->Subscribe("~/atmosphere",
      &Atmosphere::OnAtmosphereMsg, this);

  this->dataPtr->responsePub =
    this->dataPtr->node->Advertise<msgs::Response>("~/response");

  this->dataPtr->requestSub = this->dataPtr->node->Subscribe("~/request",
      &Atmosphere::OnRequest, this);
}

//////////////////////////////////////////////////
Atmosphere::~Atmosphere()
{
}

//////////////////////////////////////////////////
void Atmosphere::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->sdf->Copy(_sdf);

  this->dataPtr->temperatureSL =
      this->dataPtr->sdf->GetElement("temperature")->Get<double>();

  this->dataPtr->temperatureGradientSL =
      this->dataPtr->sdf->GetElement("temperature_gradient")->Get<double>();

  this->dataPtr->pressureSL =
      this->dataPtr->sdf->GetElement("pressure")->Get<double>();

  this->dataPtr->massDensitySL =
      this->dataPtr->sdf->GetElement("mass_density")->Get<double>();
}

//////////////////////////////////////////////////
void Atmosphere::Fini()
{
  this->dataPtr->world.reset();
  this->dataPtr->node->Fini();
}

//////////////////////////////////////////////////
sdf::ElementPtr Atmosphere::SDF() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
void Atmosphere::OnRequest(ConstRequestPtr &/*_msg*/)
{
}

//////////////////////////////////////////////////
void Atmosphere::OnAtmosphereMsg(ConstAtmospherePtr &/*_msg*/)
{
}

//////////////////////////////////////////////////
bool Atmosphere::SetParam(const std::string &_key,
    const boost::any &_value)
{
  try
  {
    if (_key == "type")
    {
      // Cannot set atmosphere model type from SetParam
      return false;
    }
    if (_key == "temperature")
      this->SetTemperatureSL(boost::any_cast<double>(_value));
    else if (_key == "pressure")
      this->SetPressureSL(boost::any_cast<double>(_value));
    else if (_key == "mass_density")
      this->SetMassDensitySL(boost::any_cast<double>(_value));
    else if (_key == "temperature_gradient")
      this->SetTemperatureGradientSL(boost::any_cast<double>(_value));
    else
    {
      gzwarn << "SetParam failed for [" << _key << "] in atmosphere model "
             << this->Type() << std::endl;
      return false;
    }
  }
  catch(boost::bad_any_cast &_e)
  {
    gzerr << "Caught bad any_cast in Atmosphere::SetParam: " << _e.what()
          << std::endl;
    return false;
  }
  catch(boost::bad_lexical_cast &_e)
  {
    gzerr << "Caught bad lexical_cast in Atmosphere::SetParam: "
          << _e.what() << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
boost::any Atmosphere::Param(const std::string &/*_key*/) const
{
  return 0;
}

//////////////////////////////////////////////////
bool Atmosphere::Param(const std::string &_key,
    boost::any &_value) const
{
  if (_key == "type")
    _value = this->Type();
  else if (_key == "temperature")
    _value = this->TemperatureSL();
  else if (_key == "pressure")
    _value = this->PressureSL();
  else if (_key == "mass_density")
    _value = this->MassDensitySL();
  else if (_key == "temperature_gradient")
    _value = this->TemperatureGradientSL();
  else
  {
    gzwarn << "Param failed for [" << _key << "] in atmosphere model "
           << this->Type() << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
double Atmosphere::TemperatureSL() const
{
  return this->Temperature(0.0);
}

//////////////////////////////////////////////////
double Atmosphere::PressureSL() const
{
  return this->Pressure(0.0);
}

//////////////////////////////////////////////////
double Atmosphere::MassDensitySL() const
{
  return this->MassDensity(0.0);
}

//////////////////////////////////////////////////
double Atmosphere::TemperatureGradientSL() const
{
  return this->dataPtr->temperatureGradientSL;
}
