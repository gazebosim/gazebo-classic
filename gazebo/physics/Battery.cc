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

#include "gazebo/common/Events.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Battery.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
Battery::Battery(LinkPtr _link)
  : dataPtr(new BatteryPrivate)
{
  this->dataPtr->link = _link;

  this->dataPtr->initVoltage = 0.0;

  this->dataPtr->initPowerLoad = 0.0;

  this->SetUpdateFunc(boost::bind(&Battery::UpdateDefault, this, _1, _2));
}

/////////////////////////////////////////////////
Battery::~Battery()
{
  this->dataPtr->link.reset();
  event::Events::DisconnectWorldUpdateEnd(this->dataPtr->connection);

  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void Battery::Load(sdf::ElementPtr _sdf)
{
  this->UpdateParameters(_sdf);

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
          boost::bind(&Battery::OnUpdate, this));
}

/////////////////////////////////////////////////
void Battery::Init()
{
  this->dataPtr->curVoltage = this->dataPtr->initVoltage;
  this->InitConsumers();
}

//////////////////////////////////////////////////
void Battery::UpdateParameters(sdf::ElementPtr _sdf)
{
  this->dataPtr->initVoltage = _sdf->Get<double>("voltage");
  this->dataPtr->initPowerLoad = _sdf->Get<double>("initial_power_load");
}

//////////////////////////////////////////////////
LinkPtr Battery::GetLink() const
{
  return this->dataPtr->link;
}

//////////////////////////////////////////////////
void Battery::InitConsumers()
{
  this->dataPtr->powerLoads.clear();
  uint32_t consumerId = this->AddConsumer();
  this->SetPowerLoad(consumerId, this->dataPtr->initPowerLoad);
}

/////////////////////////////////////////////////
uint32_t Battery::AddConsumer()
{
  uint32_t consumerId = 0;
  std::map<uint32_t, double>::const_reverse_iterator it =
    this->dataPtr->powerLoads.rbegin();
  if (it != this->dataPtr->powerLoads.rend())
  {
    consumerId = it->first + 1;
  }
  this->dataPtr->powerLoads[consumerId] = 0.0;
  return consumerId;
}

/////////////////////////////////////////////////
void Battery::RemoveConsumer(uint32_t _consumerId)
{
  this->dataPtr->powerLoads.erase(_consumerId);
}

/////////////////////////////////////////////////
void Battery::SetPowerLoad(uint32_t _consumerId, double _powerLoad)
{
  std::map<uint32_t, double>::iterator it =
    this->dataPtr->powerLoads.find(_consumerId);
  if (it != this->dataPtr->powerLoads.end())
    it->second = _powerLoad;
  else
    gzerr << "Invalid param value[_consumerId] : " << _consumerId << "\n";
}

/////////////////////////////////////////////////
double Battery::GetPowerLoad(uint32_t _consumerId) const
{
  std::map<uint32_t, double>::const_iterator it =
    this->dataPtr->powerLoads.find(_consumerId);
  if (it != this->dataPtr->powerLoads.end())
    return it->second;
  else
    gzerr << "Invalid param value[_consumerId] : " << _consumerId << "\n";
  return 0.0;
}

/////////////////////////////////////////////////
const std::map<uint32_t, double>& Battery::GetPowerLoads() const
{
  return this->dataPtr->powerLoads;
}

/////////////////////////////////////////////////
double Battery::GetVoltage() const
{
  return this->dataPtr->curVoltage;
}

/////////////////////////////////////////////////
void Battery::OnUpdate()
{
  this->dataPtr->curVoltage =
      this->dataPtr->updateFunc(this->dataPtr->curVoltage,
                                this->dataPtr->powerLoads);
}

/////////////////////////////////////////////////
double Battery::UpdateDefault(double _voltage,
    const std::map<uint32_t, double> &/*_powerLoads*/)
{
  // Ideal battery
  return _voltage;
}
