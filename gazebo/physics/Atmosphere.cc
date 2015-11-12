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

using namespace gazebo;
using namespace physics;

/// This models a base atmosphere class to serve as a common
/// interface to any derived atmosphere models.

//////////////////////////////////////////////////
Atmosphere::Atmosphere(WorldPtr _world)
  : world(_world)
  , temperatureSL(0)
  , temperatureGradientSL(0)
  , pressureSL(0)
  , massDensitySL(0)
{
  this->sdf.reset(new sdf::Element);
  sdf::initFile("atmosphere.sdf", this->sdf);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());
  this->atmosphereSub = this->node->Subscribe("~/atmosphere",
      &Atmosphere::OnAtmosphereMsg, this);

  this->responsePub =
    this->node->Advertise<msgs::Response>("~/response");

  this->requestSub = this->node->Subscribe("~/request",
                                           &Atmosphere::OnRequest, this);
}

//////////////////////////////////////////////////
Atmosphere::~Atmosphere()
{
  this->sdf->Reset();
  this->sdf.reset();
  this->responsePub.reset();
  this->requestSub.reset();
  this->node.reset();
}

//////////////////////////////////////////////////
void Atmosphere::Load(sdf::ElementPtr _sdf)
{
  this->sdf->Copy(_sdf);

  this->temperatureSL =
      this->sdf->GetElement("temperature")->Get<double>();

  this->temperatureGradientSL =
      this->sdf->GetElement("temperature_gradient")->Get<double>();

  this->pressureSL =
      this->sdf->GetElement("pressure")->Get<double>();

  this->massDensitySL =
      this->sdf->GetElement("mass_density")->Get<double>();
}

//////////////////////////////////////////////////
void Atmosphere::Fini()
{
  this->world.reset();
  this->node->Fini();
}

//////////////////////////////////////////////////
sdf::ElementPtr Atmosphere::SDF() const
{
  return this->sdf;
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
