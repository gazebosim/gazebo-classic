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
#include "gazebo/physics/World.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Pointer to private data.
    class AtmospherePrivate
    {
      /// \brief Class constructor.
      /// \param[in] _world The reference to the world.
      public: AtmospherePrivate(physics::World &_world)
        : world(_world)
      {
      }

      /// \brief Our SDF values.
      public: sdf::ElementPtr sdf;

      /// \brief Reference to the world.
      public: World &world;

      /// \brief Temperature at sea level in kelvins.
      public: double temperature = 288.15;

      /// \brief Temperature gradient at sea level in K/m.
      public: double temperatureGradient = -0.0065;

      /// \brief Pressure of the air at sea level in pascals.
      public: double pressure = 101325;

      /// \brief Mass density of the air at sea level in kg/m^3.
      public: double massDensity = 1.225;

      // Transport is declared last.
      /// \brief Node for communication.
      public: transport::NodePtr node;

      /// \brief Response publisher.
      public: transport::PublisherPtr responsePub;

      /// \brief Subscribe to the atmosphere topic.
      public: transport::SubscriberPtr atmosphereSub;

      /// \brief Subscribe to the request topic.
      public: transport::SubscriberPtr requestSub;
    };
  }
}

using namespace gazebo;
using namespace physics;

const double Atmosphere::MOLAR_MASS = 0.0289644;
const double Atmosphere::IDEAL_GAS_CONSTANT_R = 8.3144621;

//////////////////////////////////////////////////
Atmosphere::Atmosphere(physics::World &_world)
  : dataPtr(new AtmospherePrivate(_world))
{
  this->dataPtr->sdf.reset(new sdf::Element);
  sdf::initFile("atmosphere.sdf", this->dataPtr->sdf);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->world.GetName());
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
  // Must call fini on node to remove it from topic manager.
  this->dataPtr->node->Fini();
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

  this->UpdateMassDensity();
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
void Atmosphere::OnAtmosphereMsg(ConstAtmospherePtr &_msg)
{
  if (_msg->has_enable_atmosphere())
    this->World().SetAtmosphereEnabled(_msg->enable_atmosphere());

  if (_msg->has_temperature())
    this->SetTemperature(_msg->temperature());

  if (_msg->has_pressure())
    this->SetPressure(_msg->pressure());
}

//////////////////////////////////////////////////
void Atmosphere::SetTemperature(const double _temperature)
{
  this->dataPtr->temperature = _temperature;
  this->UpdateMassDensity();
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
  this->UpdateMassDensity();
}

//////////////////////////////////////////////////
double Atmosphere::Temperature(const double /*_altitude*/) const
{
  return this->dataPtr->temperature;
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
World &Atmosphere::World() const
{
  return this->dataPtr->world;
}

//////////////////////////////////////////////////
void Atmosphere::Publish(const msgs::Response &_msg) const
{
  this->dataPtr->responsePub->Publish(_msg);
}

//////////////////////////////////////////////////
void Atmosphere::UpdateMassDensity()
{
  this->dataPtr->massDensity = Atmosphere::Pressure() *
    Atmosphere::MOLAR_MASS /
    (Atmosphere::IDEAL_GAS_CONSTANT_R * Atmosphere::Temperature());
}
