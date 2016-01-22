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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <sdf/sdf.hh>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/physics/Atmosphere.hh"
#include "gazebo/physics/AtmospherePrivate.hh"
#include "gazebo/physics/World.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Atmosphere::Atmosphere(WorldPtr _world)
  : dataPtr(new AtmospherePrivate)
{
  this->dataPtr->world = _world;
  this->dataPtr->temperature = 288.15;
  this->dataPtr->temperatureGradient = -0.0065;
  this->dataPtr->pressure = 101325;
  this->dataPtr->massDensity = 1.225;

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

  if (this->dataPtr->sdf->HasElement("temperature"))
  {
    this->dataPtr->temperature =
        this->dataPtr->sdf->GetElement("temperature")->Get<double>();
  }

  if (this->dataPtr->sdf->HasElement("temperature_gradient"))
  {
    this->dataPtr->temperatureGradient =
        this->dataPtr->sdf->GetElement("temperature_gradient")->Get<double>();
  }

  if (this->dataPtr->sdf->HasElement("pressure"))
  {
    this->dataPtr->pressure =
        this->dataPtr->sdf->GetElement("pressure")->Get<double>();
  }

  if (this->dataPtr->sdf->HasElement("mass_density"))
  {
    this->dataPtr->massDensity =
        this->dataPtr->sdf->GetElement("mass_density")->Get<double>();
  }
}

//////////////////////////////////////////////////
void Atmosphere::Fini()
{
  this->dataPtr->world.reset();
  this->dataPtr->node->Fini();
}

//////////////////////////////////////////////////
void Atmosphere::Reset()
{
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
void Atmosphere::SetTemperature(const double _temperature)
{
  this->dataPtr->temperature = _temperature;
}

//////////////////////////////////////////////////
void Atmosphere::SetTemperatureGradient(const double _gradient)
{
  this->dataPtr->temperatureGradient = _gradient;
}

//////////////////////////////////////////////////
void Atmosphere::SetPressure(const double _pressure)
{
  this->dataPtr->pressure = _pressure;
}

//////////////////////////////////////////////////
double Atmosphere::Temperature(const double /*_altitude*/) const
{
  return this->dataPtr->temperature;
}

//////////////////////////////////////////////////
void Atmosphere::SetMassDensity(const double _massDensity)
{
  this->dataPtr->massDensity = _massDensity;
}

//////////////////////////////////////////////////
double Atmosphere::Pressure(const double /*_altitude*/) const
{
  return this->dataPtr->pressure;
}

//////////////////////////////////////////////////
double Atmosphere::MassDensity(const double /*_altitude*/) const
{
  return this->dataPtr->massDensity;
}

//////////////////////////////////////////////////
double Atmosphere::TemperatureGradient() const
{
  return this->dataPtr->temperatureGradient;
}

//////////////////////////////////////////////////
WorldPtr Atmosphere::World() const
{
  return this->dataPtr->world;
}

//////////////////////////////////////////////////
void Atmosphere::Publish(const msgs::Response &_msg) const
{
  this->dataPtr->responsePub->Publish(_msg);
}
